/*
 * Copyright (C) 2004-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*
Revision History:
                   Modification    
Author                 Date        Description of Changes
----------------   ------------    -------------------------
Motorola            08/09/2005     Make use of I2C on SCM-A11
Motorola            01/27/2006     Dynamically determine I2C
                                   address
Motorola            02/15/2006     Add support for Omnivision
                                   OV2640
Motorola            03/03/2006     EZXBASE 48 upmerge
Motorola            03/15/2006     Add support for Sensor 2M
Motorola            04/05/2006     Support OV2640 rev2b
Motorola            05/02/2006     Register camerai2c for tst cmd
Motorola            01/04/2007     OSS code changes

*/

#include <linux/miscdevice.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/camera.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <asm/semaphore.h>

#include <linux/sound.h>
#include <linux/soundcard.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include <asm/arch/mxc_i2c.h>

/* Major 10, Minor 244, /dev/camerai2c */
#define CAM_NAME        "cami2c"

#define TESTCODE
#undef  TESTCODE
#define I2C_A780_CAMERA 0x86

#define I2C_BUS 0 /* NOTE: bus id on SCM-A11 */

/* NOTE: Should've been defined in <linux/i2c.h> */
#define I2C_M_WR        0x00    

#define DEBUG 1

/*-----------------------------------------------------------*/
#define err_print(fmt, args...) printk(KERN_ERR "I2C-CAMERA in fun:%s "fmt"\n", __FUNCTION__, ##args)

#ifndef NDEBUG
#define dbg_print(fmt, args...) printk(KERN_INFO "I2C-CAMERA in fun:%s "fmt"\n", __FUNCTION__, ##args)
#if     DEBUG > 1
#define ddbg_print(fmt, args...) dbg_print(fmt, ##args)
#else
#define ddbg_print(fmt, args...) ;
#endif
#else
#define dbg_print(fmt, args...) ;
#define ddbg_print(fmt, args...) ;
#endif
/*-----------------------------------------------------------*/

#define I2C_CLIENT_NONE         100
#define I2C_CLIENT_SENSOR2M     101
static int i2c_camera_client_type = I2C_CLIENT_NONE;
static unsigned long i2c_camera_chipid = 0;


static int a780_camera_adapter_attach(struct i2c_adapter *adap);
static int a780_camera_detach(struct i2c_client *client);
static int a780_camera_client_register(struct i2c_client *client);
static int a780_camera_client_unregister(struct i2c_client *client);
/* ----------------------------------------------------------------------- */
static struct i2c_adapter a780_camera_adapter = {
        name:                   "a780 camera adapter",
        id:                     I2C_A780_CAMERA,
        client_register:        a780_camera_client_register,
        client_unregister:      a780_camera_client_unregister,
};

static struct i2c_client client_template =
{
    name:   "(unset)",        
    adapter:&a780_camera_adapter,
};

struct i2c_client *a780_camera_client;
unsigned int a780_camera_minor;
	
static int a780_camera_open(void)
{
#if 0
	MOD_INC_USE_COUNT;
#endif	
	return 0;
}

static int a780_camera_release(void)
{
#if 0
	MOD_DEC_USE_COUNT;
#endif
	return 0;
}


static int a780_camera_read(char *buf, size_t addr_count, size_t data_count)
{
    int ret;
 
    /* NOTE: I2C API for SCM-A11 */
    ret = mxc_i2c_read(I2C_BUS, a780_camera_client->addr, buf, addr_count, buf+addr_count, data_count);

    return ret;
			
}
	
static int a780_camera_write(const char *buf, size_t addr_count, size_t data_count)
{
	int ret;

	/* NOTE: I2C API for SCM-A11 */
	ret = mxc_i2c_write(I2C_BUS, a780_camera_client->addr, \
			    (char *)buf, addr_count, (char *)(buf+addr_count), data_count);

	return ret;
}

int a780_camera_transfer(struct i2c_msg msgs[],int num)
{
   int ret;

   a780_camera_open();

   ret = i2c_transfer(a780_camera_client->adapter, msgs, num);

   a780_camera_release();
   return ret;
}

static int a780_camera_client_register(struct i2c_client *client)
{
	
	return 0;
}

static int a780_camera_client_unregister(struct i2c_client *client)
{
	
	return 0;	
}
/* ----------------------------------------------------------------------- */

static int a780_camera_adapter_attach(struct i2c_adapter *adap)
{
	if(! (a780_camera_client = kmalloc(sizeof(struct i2c_client),GFP_KERNEL)))
		return -ENOMEM;
	memcpy(a780_camera_client,&client_template,sizeof(struct i2c_client));
	a780_camera_client->adapter = adap;
        
	/* NOTE: just a default value; will be overwritten later */
	a780_camera_client->addr = 0x5D;
	
	return 0;
}	

static int a780_camera_detach(struct i2c_client *client)
{	
        kfree(a780_camera_client);

	return 0;
}
/* ----------------------------------------------------------------------- */
static int cam_open(struct inode *inode, struct file *file)
{
        if(i2c_camera_client_type == I2C_CLIENT_NONE)
            return -EINVAL;

#if 0
        MOD_INC_USE_COUNT;
#endif
        return 0;
}

static int i2c_camera_readw(unsigned short addr, unsigned short *pvalue);
static int i2c_camera_readb(unsigned short addr, unsigned char *pvalue);
static int i2c_camera_writew(unsigned short addr, unsigned short value);
static int i2c_camera_writeb(unsigned short addr, unsigned char value);

static int cam_close(struct inode * inode, struct file *file)
{
#if 0
        MOD_DEC_USE_COUNT;
#endif
        return 0;
}

#define DETECT_BUFLEN 256
static int cam_ioctl_detectid (void * arg)
{
    int    buflen, idlen;
    char*  id;
    struct camera_i2c_detectid * param = arg;
    if(copy_from_user(&buflen, &(param->buflen), sizeof(buflen)))
    {
        return -EFAULT;
    }
    if(buflen > DETECT_BUFLEN)
    {
        return -ENOMEM;
    }
    id = kmalloc(DETECT_BUFLEN, GFP_KERNEL);
    if(id == NULL)
    {
        return -ENOMEM;
    }

    idlen = 0;
    switch(i2c_camera_client_type)
    {
#ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
        case I2C_CLIENT_SENSOR2M:
            idlen = snprintf(id, DETECT_BUFLEN-1, "%s %s %lx", 
                            "SENSOR", "2M", i2c_camera_chipid);
            break;
#endif
        default:
            break;
    }
    id[DETECT_BUFLEN-1] = 0;
    idlen = strlen(id)+1;
    if(buflen < idlen)
    {
        kfree(id);
        return -ENOMEM;
    }
    if(copy_to_user(param->data, id, idlen))
    {
        kfree(id);
        return -EFAULT;
    }
    kfree(id);
    return 0;
}

static int cam_ioctl_register_rw (unsigned int cmd, void * arg)
{
    int ret = -ENOTSUPP;
    struct camera_i2c_register reg;
    if(copy_from_user(&reg, arg, sizeof(reg)))
    {
        return -EFAULT;
    }
    switch(cmd)
    {
        case CAMERA_I2C_WRITEW:
            ret=i2c_camera_writew(reg.addr, reg.value.w);
            break;
        case CAMERA_I2C_WRITEB:
            ret=i2c_camera_writeb(reg.addr, reg.value.b);
            break;
        case CAMERA_I2C_READW:
            if((ret=i2c_camera_readw(reg.addr, &(reg.value.w)))>=0)
            {
                if(copy_to_user(arg, &reg, sizeof(reg)))
                    ret = -EFAULT;
            }
            break;
        case CAMERA_I2C_READB:
            if((ret=i2c_camera_readb(reg.addr, &(reg.value.b)))>=0)
            {
                if(copy_to_user(arg, &reg, sizeof(reg)))
                    ret = -EFAULT;
            }
            break;
        default:
            break;
    }
    return ret;
}

static int cam_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = -ENOTSUPP;
    switch (cmd)
    {
        case CAMERA_I2C_WRITEW:
        case CAMERA_I2C_WRITEB:
        case CAMERA_I2C_READW:
        case CAMERA_I2C_READB:
            ret = cam_ioctl_register_rw(cmd, (void *)arg);
            break;
        case CAMERA_I2C_DETECTID:
            ret = cam_ioctl_detectid((void *)arg);
            break;
        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

static struct file_operations cam_fops = {
        ioctl:          cam_ioctl,
        open:           cam_open,
        release:        cam_close,
};

static struct miscdevice cam_misc_device = {
        minor: CAM_MINOR,
        name: CAM_NAME,
        fops: &cam_fops,
	devfs_name: "camerai2c",
};

/* ----------------------------------------------------------------------- */
static int a780_camera_init_module(void)
{
	int res;
	
	res = a780_camera_adapter_attach(&a780_camera_adapter);
	if( res < 0 )
	{
		printk("error in camera_adapter_attach\n");
		return res;
	}
	if (misc_register (&cam_misc_device))
	  {
	    printk(KERN_ERR "Couldn't register cam driver\n");
	    return -EIO;
	  }

	return 0;
}

static void a780_camera_cleanup_module(void)
{	
        a780_camera_detach(a780_camera_client);
	misc_deregister(&cam_misc_device);
}


#ifdef  CONFIG_MXC_IPU_CAMERA_SENSOR2M
#define SENSOR2M_CHIP_ID_REG                    0x3000
#define SENSOR2M_CHIP_ID_VALUE                  0x1580
static const unsigned char sensor2m_i2c_base_addr[4]={0x7A>>1, 0x78>>1, 0xBA>>1, 0x90>>1};

int i2c_sensor2m_read(unsigned short addr, unsigned short *pvalue)
{
  int ret;
  char tmp[4] = {(char)(addr>>8), (char)(addr&0x00FF), 0xFF, 0xFF};

  ret = a780_camera_read(tmp, 2, 2);
  if(ret<=0)
    {
      err_print("read error! address=%04x, return code=%d", addr, ret);
      return -1;
    }

  *pvalue = (((unsigned short)(tmp[2])) << 8) | tmp[3];

  return 0;
}

int i2c_sensor2m_write(unsigned short addr, unsigned short value)
{
  int ret;
  char tmp[4] = {(char)(addr>>8), (char)(addr&0x00FF), (char)(value>>8), (char)(value&0x00FF)};

  ret = a780_camera_write(tmp, 2, 2);
  if(ret <= 0)
    {
      err_print("write error! address=%04x, return code=%d", addr, ret);
      return -1;
    }

  return 0;
}

int i2c_sensor2m_init(void)
{
  int ret;
  int i;
  unsigned short chipid;
  ddbg_print("open!");

  for(i=0;i<4;i++)
    {
      a780_camera_client->addr = sensor2m_i2c_base_addr[i];

      ret = i2c_sensor2m_read(SENSOR2M_CHIP_ID_REG, &chipid);

      if(ret==0)
	{
	  break;
	}
    }

  if(i==4)
    {
      err_print("error: failed to read chipid");
      return -EIO;
    }


  ddbg_print("sensor2m chip id is 0x%x", chipid);

  i2c_camera_client_type = I2C_CLIENT_SENSOR2M;
  i2c_camera_chipid = chipid;
  return 0;
}

int i2c_sensor2m_cleanup(void)
{
  ddbg_print("close!");
  i2c_camera_client_type = I2C_CLIENT_NONE;
  return 0;
}

#endif

static int i2c_camera_readw(unsigned short addr, unsigned short *pvalue)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
#ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
        case I2C_CLIENT_SENSOR2M:
            ret = i2c_sensor2m_read(addr, pvalue);
            break;
#endif
        default:
            break;
    }
    return ret;
}

static int i2c_camera_readb(unsigned short addr, unsigned char *pvalue)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
        default:
            break;
    }
    return ret;
}

static int i2c_camera_writew(unsigned short addr, unsigned short value)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
#ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
        case I2C_CLIENT_SENSOR2M:
            ret = i2c_sensor2m_write(addr, value);
            break;
#endif
        default:
            break;
    }
    return ret;
}

static int i2c_camera_writeb(unsigned short addr, unsigned char value)
{
    int ret = -ENOTSUPP;
    switch(i2c_camera_client_type)
    {
        default:
            break;
    }
    return ret;
}

#ifdef CONFIG_MXC_IPU_CAMERA_SENSOR2M
EXPORT_SYMBOL(i2c_sensor2m_init);
EXPORT_SYMBOL(i2c_sensor2m_write);
EXPORT_SYMBOL(i2c_sensor2m_read);
EXPORT_SYMBOL(i2c_sensor2m_cleanup);
#endif

module_init(a780_camera_init_module);
module_exit(a780_camera_cleanup_module);
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
