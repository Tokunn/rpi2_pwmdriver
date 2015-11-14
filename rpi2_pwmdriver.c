#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>

#include "rpi_gpio.h"

MODULE_AUTHOR("Tokunn");
MODULE_LICENSE("Dual BSD/GPL");

#define PWMDRV_NUM_DEVS	4			/* このドライバが制御するデバイスの数 */
#define PWMDRV_DEVNAME	"pwm_driver"	/* このデバイスドライバの名称 */
#define PWMDRV_MAJOR		0			/* メジャー番号だが自動設定なので0 */
#define PWMDRV_MINOR		0			/* マイナー番号のベース番号 */

#define PDRV_GPIO_MAPNAME	"pwmdrv_gpio_map"

static int _pwm_driver_major = PWMDRV_MAJOR;
static int _pwm_driver_minor = PWMDRV_MINOR;

static struct cdev *pwm_driver_cdev_array = NULL;
static struct class *pwm_driver_class = NULL;

static spinlock_t pwm_driver_spinlock;
static int ref_counter[PWMDRV_NUM_DEVS] = {0,0,0,0};

struct pwm_driver_info {
	int minor;
	int *duty_ratio;
    //int *duty;
};


static void __iomem *gpio_map;
static volatile uint32_t *gpio_base;

#define LED_BASE	7
#define DIGIT_BASE	14
#define DIGITS		4
#define BLANK		10
#define	DEFAULT_PWM_HZ	1000
#define DEFAULT_PWM_PERIOD_NSEC 1 * 1000 * 1000 * 1000 / DEFAULT_PWM_HZ

static struct hrtimer refresh_timer;
static int pwm_hz = DEFAULT_PWM_HZ;
static int pwm_period_nsec = DEFAULT_PWM_PERIOD_NSEC;
static int display_digits = DIGITS;
#define REFRESH_KTIME	 ktime_set( 0, (1000000000/DIGITS) / pwm_hz )

static int duty_ratio_list[DIGITS] = {40,30,20,10};
static int duty_list[DIGITS] = {0};
static int pinnum_list[DIGITS] = {0,1,2,3};

static int list_size = DIGITS;
static int duty_next_counter = 0;

/* proto types */
static int rpi_gpio_map(void);
static int rpi_gpio_unmap(void);
static int rpi_gpio_function_set(int, uint32_t);
static void rpi_gpio_set32( uint32_t, uint32_t );
static void rpi_gpio_clear32( uint32_t, uint32_t );
static void gpio_setup(void);
//static void led_put(unsigned int);
static void register_refresh_timer(void);
static enum hrtimer_restart refresh_timer_handler(struct hrtimer *);
static int pwm_driver_init(void);
static void pwm_driver_exit(void);

/*----- print_lists() -----*/
void print_lists(int *p_lists, int p_size) {
    int i;
    for (i = 0; i < p_size; i++) {
        printk(KERN_INFO "%d", p_lists[i]);
    }
}

/*----- sort_function() -----*/
static void sort_function(int *p_mainlist, int *p_sublist) {
    int i,j,maintmp,subtmp;
    for (i = 0; i < list_size -1; i++) {
        for (j = list_size - 1; j > i; j--) {
            if ( p_mainlist[j] < p_mainlist[j-1]) {
                maintmp = p_mainlist[j];
                p_mainlist[j] = p_mainlist[j-1];
                p_mainlist[j-1] = maintmp;

                subtmp = p_sublist[j];
                p_sublist[j] = p_sublist[j-1];
                p_sublist[j-1] = subtmp;
            }
        }
    }
}

static int rpi_gpio_map(void)
{
	if( !request_mem_region(RPI_GPIO_BASE,
							RPI_BLOCK_SIZE,
							PDRV_GPIO_MAPNAME) ) {
		printk( KERN_ALERT "request_mem_region failed.\n");
		return -EBUSY;
	}
	gpio_map = ioremap_nocache(RPI_GPIO_BASE, BLOCK_SIZE);
	gpio_base = (volatile uint32_t *)gpio_map;
	
	return 0;
}

static int rpi_gpio_unmap(void)
{
	iounmap(gpio_map);
	release_mem_region(RPI_GPIO_BASE, RPI_BLOCK_SIZE);

	gpio_map = NULL;
	gpio_base = NULL;
	return 0;
}

static int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));
	gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));
	
	return 1;
}

static void rpi_gpio_set32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

static void rpi_gpio_clear32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

static void gpio_setup(void)
{
	rpi_gpio_function_set( LED_BASE+0, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED_BASE+1, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED_BASE+2, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( LED_BASE+3, RPI_GPF_OUTPUT );

	rpi_gpio_function_set( DIGIT_BASE+0, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( DIGIT_BASE+1, RPI_GPF_OUTPUT );
}

/*static void led_put( unsigned int v )
{
	rpi_gpio_clear32( RPI_GPIO_P1MASK, 0x0F << LED_BASE);
	rpi_gpio_set32( RPI_GPIO_P1MASK, (v & 0x0F) << LED_BASE);	
}*/

static void register_refresh_timer(void)
{
	if( (pwm_hz < 1) || (pwm_hz >= 1000000) )
		pwm_hz = DEFAULT_PWM_HZ;
	
	hrtimer_init(&refresh_timer, HRTIMER_MODE_ABS, HRTIMER_MODE_REL);
	refresh_timer.function = refresh_timer_handler;
	hrtimer_start(&refresh_timer, ktime_set(0, pwm_period_nsec), HRTIMER_MODE_REL );
}

static enum hrtimer_restart refresh_timer_handler(struct hrtimer *timer)
{
	static int dig = 0;
    int i,next_timer_nsec;
	ktime_t now;
	
	if( ++dig >= DIGITS ) dig = 0;
	
	/*led_put(BLANK);
	rpi_gpio_clear32( RPI_GPIO_P1MASK, 0x03 << DIGIT_BASE);
	rpi_gpio_set32( RPI_GPIO_P1MASK, (dig) << DIGIT_BASE);
	led_put(duty_ratio_list[dig]);*/
	
	if( (pwm_hz < 1) || (pwm_hz >= 1000000) )
		pwm_hz = DEFAULT_PWM_HZ;

    // Pulse start (origin = time zero)
    if (duty_next_counter == 0) {
        for (i = 0; i < list_size; i++) {
            if (duty_list[i]) {
                // TODO PIN[i] = HIGH
                //rpi_gpio_set32( RPI_GPIO_P1MASK, (dig) << DIGIT_BASE);
                printk("PIN[%d] = HIGH\n", i);
            }
        }
        next_timer_nsec = duty_list[duty_next_counter];
        duty_next_counter++;
    }
    // List last (time most closer to the end)
    else if (duty_next_counter >= list_size) { // TODO list_size or list_size-1
        // TODO PIN[pinnum_list[duty_next_counter-1]] = LOW
        printk("PIN[%d] = LOW\n", pinnum_list[duty_next_counter-1]);
        //rpi_gpio_clear32( RPI_GPIO_P1MASK, 0x03 << DIGIT_BASE);
        next_timer_nsec = pwm_period_nsec - duty_list[duty_next_counter-1];
        duty_next_counter = 0;
    }
    else {
        // TODO PIN[pinnum_list[duty_next_counter-1]] = LOW
        printk("PIN[%d] = LOW\n", pinnum_list[duty_next_counter-1]);
        //rpi_gpio_clear32( RPI_GPIO_P1MASK, 0x03 << DIGIT_BASE);
        next_timer_nsec = duty_list[duty_next_counter] - duty_list[duty_next_counter-1];
        duty_next_counter++;
    }

    if (next_timer_nsec < 1) {
        next_timer_nsec = 1;
    }
	
	now = ktime_get();
    hrtimer_forward(timer, now, ktime_set(0, next_timer_nsec) );
    return HRTIMER_RESTART;
}


static int pwm_driver_open(struct inode *inode, struct file *filep)
{
	int retval;
	int minor = MINOR(inode->i_rdev);
	struct pwm_driver_info *info;
	
	spin_lock(&pwm_driver_spinlock);
	if( ref_counter[minor] == 0 ) {
		ref_counter[minor]++;
		
		info = (struct pwm_driver_info *)kmalloc(sizeof(struct pwm_driver_info), GFP_KERNEL);
		info->minor = minor;
		info->duty_ratio = &(duty_ratio_list[minor]);
        //info->duty = &(duty_list[minor]);
		filep->private_data = (void *)info;
		retval = 0;
	}
	else {
		retval = -EBUSY;
	}
	spin_unlock(&pwm_driver_spinlock);
	
	return retval;
}

static int pwm_driver_release(struct inode *inode, struct file *filep)
{
	struct pwm_driver_info *info = (struct pwm_driver_info *)filep->private_data;

	spin_lock(&pwm_driver_spinlock);
	ref_counter[info->minor]--;
	kfree(info);
	spin_unlock(&pwm_driver_spinlock);
	
	return 0;
}


static ssize_t pwm_driver_write(
	struct file *filep,
	const char __user *buf,
    size_t count,
    loff_t *f_pos)
{
	struct pwm_driver_info *info = (struct pwm_driver_info *)filep->private_data;
    int i,j;
	
	if( (pwm_hz < 1) || (pwm_hz >= 1000000) )
		pwm_hz = DEFAULT_PWM_HZ;

	if(count > 0) {
		if(copy_from_user( info->duty_ratio, buf, sizeof(char) )) {
			return -EFAULT;
		}
        list_size = DIGITS;
        pwm_period_nsec = 1 * 1000 * 1000 * 1000 / pwm_hz;

        /**info->duty = *info->duty_ratio * pwm_period_nsec / 100;
	    printk(KERN_INFO "%s: *info->duty = %d\n", PWMDRV_DEVNAME, *info->duty);
        print_lists(duty_list, list_size);*/

        for (i = 0; i < list_size; i++) {
            duty_list[i] = duty_ratio_list[i] * pwm_period_nsec / 100;
        }

        printk(KERN_INFO "%s: duty_ratio_list[0] = %d\n", PWMDRV_DEVNAME, duty_ratio_list[0]);
        printk(KERN_INFO "%s: duty_ratio_list[1] = %d\n", PWMDRV_DEVNAME, duty_ratio_list[1]);
        printk(KERN_INFO "%s: duty_ratio_list[2] = %d\n", PWMDRV_DEVNAME, duty_ratio_list[2]);
        printk(KERN_INFO "%s: duty_ratio_list[3] = %d\n", PWMDRV_DEVNAME, duty_ratio_list[3]);

        printk(KERN_INFO "%s: duty_list[0](raw) = %d\n", PWMDRV_DEVNAME, duty_list[0]);
        printk(KERN_INFO "%s: duty_list[1](raw) = %d\n", PWMDRV_DEVNAME, duty_list[1]);
        printk(KERN_INFO "%s: duty_list[2](raw) = %d\n", PWMDRV_DEVNAME, duty_list[2]);
        printk(KERN_INFO "%s: duty_list[3](raw) = %d\n", PWMDRV_DEVNAME, duty_list[3]);

        sort_function(duty_list, pinnum_list);
        for (i = 0; i < list_size-1; i++) {
            if (!duty_list[i]) {
                for (j = i; j < list_size-1; j++) {
                    duty_list[j] = duty_list[j+1];
                    pinnum_list[j] = pinnum_list[j+1];
                }
                list_size--;
            }
        }
        printk(KERN_INFO "%s: list_size=%d\n", PWMDRV_DEVNAME, list_size);

        printk(KERN_INFO "%s: duty_list[0](alignment) = %d\n", PWMDRV_DEVNAME, duty_list[0]);
        printk(KERN_INFO "%s: duty_list[1](alignment) = %d\n", PWMDRV_DEVNAME, duty_list[1]);
        printk(KERN_INFO "%s: duty_list[2](alignment) = %d\n", PWMDRV_DEVNAME, duty_list[2]);
        printk(KERN_INFO "%s: duty_list[3](alignment) = %d\n", PWMDRV_DEVNAME, duty_list[3]);

        printk(KERN_INFO "\n");
		return sizeof(char);
	}
	return 0;
}


struct file_operations pwm_driver_fops = {
	.open      = pwm_driver_open,
	.release   = pwm_driver_release,
	.write     = pwm_driver_write,
};


static int pwm_driver_register_dev(void)
{
	int retval;
	dev_t dev;
	size_t size;
	int i;
	
	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,				/* 結果を格納するdev_t構造体 */
		PWMDRV_MINOR,		/* ベースマイナー番号 */
		PWMDRV_NUM_DEVS,	/* デバイスの数 */
		PWMDRV_DEVNAME		/* デバイスドライバの名前 */
	);
	
	if( retval < 0 ) {
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_pwm_driver_major = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	pwm_driver_class = class_create(THIS_MODULE,PWMDRV_DEVNAME);
	if(IS_ERR(pwm_driver_class))
		return PTR_ERR(pwm_driver_class);
	
	/* cdev構造体の用意 */
	size = sizeof(struct cdev) * PWMDRV_NUM_DEVS;
	pwm_driver_cdev_array =  (struct cdev*)kmalloc(size, GFP_KERNEL);
	
	/* デバイスの数だけキャラクタデバイスを登録する */
	/* ただし7セグLEDは1個しかない */
	for( i = 0; i < PWMDRV_NUM_DEVS; i++ ) {
		dev_t devno = MKDEV(_pwm_driver_major, _pwm_driver_minor+i);
		/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
		cdev_init(&(pwm_driver_cdev_array[i]), &pwm_driver_fops);
		pwm_driver_cdev_array[i].owner = THIS_MODULE;
		if( cdev_add( &(pwm_driver_cdev_array[i]), devno, 1) < 0 ) {
			/* 登録に失敗した */
			printk(KERN_ERR "cdev_add failed minor = %d\n", _pwm_driver_minor+i );
		}
		else {
			/* デバイスノードの作成 */
			device_create(
					pwm_driver_class,
					NULL,
					devno,
					NULL,
					PWMDRV_DEVNAME"%u",_pwm_driver_minor+i
			);
		}
	}
	return 0;
}

static int pwm_driver_init(void)
{
	int retval;
	
	/* 開始のメッセージ */
	printk(KERN_INFO "%s: loading...\n", PWMDRV_DEVNAME );
	
	/* GPIOレジスタがマップ可能か調べる */
	retval = rpi_gpio_map();
	if( retval != 0 ) {
		printk( KERN_ALERT "Can not use GPIO registers.\n");
		return -EBUSY;
	}
	/* GPIO初期化 */
	gpio_setup();
	
	if( pwm_driver_register_dev() != 0 ) {
		printk( KERN_ERR "Can not register ssled4d\n");
		return -EBUSY;
	}
	/* スピンロック初期化 */
	spin_lock_init(&pwm_driver_spinlock);
	register_refresh_timer();

	printk( KERN_INFO "pwm_driver: driver register sccessed.\n");
	
	return 0;
}

static void pwm_driver_exit(void)
{
	int i;
	dev_t devno;
	
	/* キャラクタデバイスの登録解除 */
	for( i = 0; i < PWMDRV_NUM_DEVS; i++ ) {
		cdev_del(&(pwm_driver_cdev_array[i]));
		devno = MKDEV(_pwm_driver_major, _pwm_driver_minor+i);
		device_destroy(pwm_driver_class, devno);
	}
	/* メジャー番号/マイナー番号を取り除く */
	devno = MKDEV(_pwm_driver_major,_pwm_driver_minor);
	unregister_chrdev_region(devno, PWMDRV_NUM_DEVS);	
	class_destroy( pwm_driver_class );

	hrtimer_cancel(&refresh_timer);
	rpi_gpio_unmap();
	kfree(pwm_driver_cdev_array);
}

module_init(pwm_driver_init);
module_exit(pwm_driver_exit);

module_param( pwm_hz, int, S_IRWXU | S_IRGRP | S_IROTH |  S_IWUSR );
module_param( list_size, int, S_IRUSR | S_IRGRP | S_IROTH |  S_IWUSR );
module_param_array( duty_ratio_list, int, &display_digits ,S_IRUSR | S_IRGRP | S_IROTH |  S_IWUSR );
module_param_array( duty_list, int, &display_digits ,S_IRUSR | S_IRGRP | S_IROTH |  S_IWUSR );
module_param_array( pinnum_list, int, &display_digits ,S_IRUSR | S_IRGRP | S_IROTH |  S_IWUSR );
