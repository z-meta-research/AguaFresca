#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <glob.h>
#include <linux/i2c-dev.h>
#include <linux/limits.h>

#include "bme280.h"

int i2c_fd = -1;

int8_t
i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	if (write(i2c_fd, &reg_addr, sizeof(reg_addr)) < sizeof(reg_addr))
		return BME280_E_COMM_FAIL;
	if (read(i2c_fd, data, len) < len)
		return BME280_E_COMM_FAIL;
	return BME280_OK;
}

void
delay_ms(uint32_t period)
{
	usleep(period * 1000);
}

int8_t i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t *buf;
	buf = malloc(len + 1);
	buf[0] = reg_addr;
	memcpy(buf + 1, data, len);
	if (write(i2c_fd, buf, len + 1) < len)
		return BME280_E_COMM_FAIL;
	free(buf);
	return BME280_OK;
}

int
vsystem(const char *fmt, ...)
{
  char buffer[PATH_MAX];

  va_list args;
  va_start (args, fmt);
  
  vsprintf(buffer, fmt, args);

  va_end(args);

  printf(buffer); printf("\n");
  system(buffer);
}

int
main(int argc, char **argv)
{
  int i;
  int result;
  struct bme280_dev dev;
  struct bme280_data comp_data;
  glob_t dglob;
  char fnbuf[PATH_MAX + 1];
  char fn_pwm_heat[PATH_MAX + 1];
  char fn_PWM_cool[PATH_MAX + 1];
  int pwm_no = -1;
  
  double pid_p, pid_i, pid_d;
  double desired_temp;
  double last_error, error_int;
  
  pid_p = .20;
  pid_i = .02;
  pid_d = .1;

  desired_temp = 30.0;

  dglob.gl_offs = 0;

#define PWMBASE "/sys/class/pwm/pwmchip"
#define PWMPERIOD (1000000000 / 1500)
  
  result = glob(PWMBASE "*/device", 0,
		NULL, &dglob);

  for (i = 0; i < dglob.gl_pathc; i++)  {
    ssize_t s = readlink(dglob.gl_pathv[i], fnbuf, PATH_MAX);
    fnbuf[s] = 0;
    printf("%s->%s\n", dglob.gl_pathv[i], fnbuf);
    if (!strcmp(fnbuf, "../../../48300200.pwm")) {
      pwm_no = dglob.gl_pathv[i][strlen(PWMBASE)] - '0';
      printf("pwm_no: %d\n", pwm_no);
      break;
    }
  }

  globfree(&dglob);

  if (pwm_no == -1) {
    fprintf(stderr, "Cannot find pwm device\n");
    exit(1);
  }

  vsystem("echo %d > /sys/class/pwm/pwm-%d:0/duty_cycle", 0, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:0/period", PWMPERIOD, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:0/polarity", 1, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:0/enable", 1, pwm_no);

  vsystem("echo %d > /sys/class/pwm/pwm-%d:1/duty_cycle", 0, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:1/period", PWMPERIOD, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:1/polarity", 1, pwm_no);
  vsystem("echo %d > /sys/class/pwm/pwm-%d:1/enable", 1, pwm_no);

  
  dev.dev_id = BME280_I2C_ADDR_SEC;
  dev.intf = BME280_I2C_INTF;
  dev.read = i2c_read;
  dev.write = i2c_write;
  dev.delay_ms = delay_ms;

  i2c_fd = open("/dev/i2c-2", O_RDWR);

  if (i2c_fd < 0) {
    fprintf(stderr, "Failed to open i2c bus: %s\n", strerror(errno));
    exit(1);
  }

  if (ioctl(i2c_fd, I2C_SLAVE, dev.dev_id) < 0) {
    fprintf(stderr, "Failed to acquire bus access and/or talk to slave.\n");
    exit(1);
  }
	
  result = bme280_init(&dev);
  if (result != BME280_OK) {
    fprintf(stderr, "Failed to initialize the device (code %+d: %s).\n", result, strerror(-result));
    exit(1);
  }

  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;

  result = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);
  if (result != BME280_OK) {
    fprintf(stderr, "Failed to set sensor settings (code %+d: %s).\n", result, strerror(-result));
    exit(1);
  }

  result = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);  

  if (result != BME280_OK) {
    fprintf(stderr, "Failed to set sensor mode (code %+d: %s).\n", result, strerror(-result));
    exit(1);
  }

  /* Wait for the measurement to complete and print data @25Hz */
  dev.delay_ms(100);
  result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

  if (result != BME280_OK) {
    fprintf(stderr, "Failed to read data (code %+d: %s).\n", result, strerror(-result));
    exit(1);
  }

  last_error = (desired_temp - comp_data.temperature);
  
  while (1) {
    double pv, error, error_d;
    
    /* Wait for the measurement to complete and print data @25Hz */
    dev.delay_ms(100);
    result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    if (result != BME280_OK) {
      fprintf(stderr, "Failed to read data (code %+d: %s).\n", result, strerror(-result));
      continue;
    }

    error = (desired_temp - comp_data.temperature);
    error_int += error;
    error_d = error - last_error;
    last_error = error;
    
    pv = pid_p * error + pid_i * error_int + pid_d * error_d;

    if (pv > 12.0) {
      pv = 12.0;
    }

    if (pv < -12.0) {
      pv = -12.0;
    }
    
    printf("%0.2f,%0.2f,%0.2f,%0.2f,%f,%f,%f,%f\r\n",
	   comp_data.temperature, comp_data.pressure, comp_data.humidity,
	   desired_temp, error, error_int, error_d,
	   pv);

    if (pv >= 0) {
      vsystem("echo %d > /sys/class/pwm/pwm-%d:0/duty_cycle", (int)(PWMPERIOD * pv), pwm_no);
      vsystem("echo %d > /sys/class/pwm/pwm-%d:1/duty_cycle", 0, pwm_no);
    } else {
      vsystem("echo %d > /sys/class/pwm/pwm-%d:0/duty_cycle", 0, pwm_no);
      vsystem("echo %d > /sys/class/pwm/pwm-%d:1/duty_cycle", (int)(PWMPERIOD * pv), pwm_no);
    }
  }
}
