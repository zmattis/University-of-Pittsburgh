/**
 * @author Zachary M. Mattis
 * CS 0449
 * Pi Linux Device Driver
 * July 26, 2017
 *
 * This C file provides the file operations for the
 * /dev/pi device.
 */

/* Libraries */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <asm/uaccess.h> // For copy_to_user
#include "pi.h" // For pi function

MODULE_LICENSE("GPL");
MODULE_AUTHOR("encrypstream <zachary.mattis@pitt.edu>");
MODULE_DESCRIPTION("Digits of pi module");
MODULE_VERSION("dev");

/*
 * ---------------------------------------------------------------
 * pi_read is the function called when a process calls read() on
 * /dev/pi.  It writes n digits of pi to the buffer passed in the
 * read() call.
 * ---------------------------------------------------------------
 */
static ssize_t pi_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
  unsigned int allocate; // Used to work with the pi function.
  char *pi_buffer;

  allocate = (4 - (*ppos + count) % 4) + *ppos + count; // Round up to the next multiple of 4

  pi_buffer = (char*)kmalloc(allocate, GFP_KERNEL); // Allocate some extra space based on the rounded value

  /* This pi function assumes that you are giving it a buffer and the number of digits to calculate
   * You can use ANY pi function with this, as it is used via an include up above.
   * A version that does 4 digits at a time was used, hence the multiples of 4.
   */
  pi(pi_buffer, allocate);

  /* Copies to the user space, but also verifies permissions and such, which is safer
   * than simply writing directly to a user buffer. But this limits the char device to the
   * amount of space available to kmalloc.
   */
  if (copy_to_user(buf, pi_buffer + *ppos, count)) {
    kfree(pi_buffer);
    return -EINVAL;
  }

  kfree(pi_buffer); // Free the buffer
  *ppos += count; // Increment the position
  return count;
}

/* Set up what file functions can be performed
 * Only read is defined.
 */
static const struct file_operations pi_fops = {
  .owner		= THIS_MODULE,
  .read		  = pi_read,
};

static struct miscdevice pi_driver = {
  // Assign random
  MISC_DYNAMIC_MINOR,
  // Call it pi
  "pi",
  // Define file operations for pi
  &pi_fops
};

/*
 * -----------------------------------------------------------
 * Create the "pi" device in the /sys/class/misc directory.
 * Udev will automatically create the /dev/pi device using
 * the default rules.
 * -----------------------------------------------------------
 */
static int __init pi_init(void)
{
  int ret;
  ret = misc_register(&pi_driver);
  if (ret) // If for some reason it couldn't register
    printk(KERN_ERR "Unable to register pi char device\n");

  return ret;
}

module_init(pi_init);

/* Sets up /sys/class/misc/pi
 * Must manually create /dev/pi
 */
static void __exit pi_exit(void)
{
  misc_deregister(&pi_driver);
}

module_exit(pi_exit);
