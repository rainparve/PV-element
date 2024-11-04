#include <stdio.h>
#include <string.h>
#include "saml21_backup_mode.h"

#include "periph/i2c.h"
#include "periph/gpio.h"
//#include "periph/rtc.h"
#include "periph/adc.h"
#include "periph_conf.h"
#include "rtc_utils.h"

#include "hdc3020.h"
#include "hdc3020_params.h"

#include "fram.h"
#include "senseair.h"
#include "senseair_params.h"

#define FW_VERSION "01.00.00"


//Kui kasutatakse ACME Sensor 2, siis EXTWAKE_NONE
/* use { .pin=EXTWAKE_NONE } to disable */
#define EXTWAKE { \
    .pin = EXTWAKE_PIN6, \
    .polarity = EXTWAKE_HIGH, \
    .flags = EXTWAKE_IN_PU }

#define SLEEP_TIME (10) /* in seconds; -1 to disable */

#define ADC_VCC 0
#define ADC_VPANEL 1
#define VPANEL_ENABLE GPIO_PIN(PA, 27)

static saml21_extwake_t extwake = EXTWAKE;
static hdc3020_t hdc3020;
static senseair_t dev;          // Ei pruugi olla parim nimi!-------------------------------
static senseair_abc_data_t abc_data;

#define SENSEAIR_STATE_FRAM_ADDR    0
#define USES_FRAM                   (1)

static struct {
    uint8_t cpuid[CPUID_LEN];
    int32_t vcc;            //Kondensaatori pinge
    int32_t vpanel;         //PV-elemendi pinge
    double temp;            //HDC3020
    double hum;             //HDC3020
    uint16_t conc_ppm;      //Senseair
    int16_t temp_cC;        //Senseair
} measures;

void hdc3020_sensor_read(void)
{
//    double temperature, humidity = -999;    // pole otseselt vajalik. Vb algväärtustada ainult koodi alguses?

    if (hdc3020_init(&hdc3020, hdc3020_params) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL INIT.");
        return;
    }

    if (hdc3020_deactivate_alert(&hdc3020) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL DEACTIVATE ALERT.");
        return;
    }

    if (hdc3020_read(&hdc3020, &measures.temp, &measures.hum) != HDC3020_OK) {
        puts("[SENSOR hdc3020] FAIL READ.");
        return;
    }
    hdc3020_deinit(&hdc3020);

}

void seaseair_sensor_read(void)
{
//    uint16_t conc_ppm;
//    int16_t temp_cC;

    if (gpio_init(ACME_BUS_POWER_PIN, GPIO_OUT)) {
        puts("ACME Bus enable failed.");
        return;
    }
    gpio_set(ACME_BUS_POWER_PIN);

    if (senseair_init(&dev, &senseair_params[0]) != SENSEAIR_OK) {
        puts("Senseair init failed.");
        gpio_clear(ACME_BUS_POWER_PIN);
        return;
    }

    memset(&abc_data, 0, sizeof(abc_data));
    if (fram_read(SENSEAIR_STATE_FRAM_ADDR, &abc_data, sizeof(abc_data))) {
        puts("FRAM read failed.");
    }
    else {
        if (senseair_write_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
            puts("ABC data restored to sensor.");
        }
        else {
            puts("ABC data not available.");
        }
    }

// Siin ehk peaks temperatuuri salvestama koheselt õigeks formaadiks.
// senseair_read võib saada ajutise temperatuuri aadressi ja struktuuri salvestatakse juba jagatud väärtus.
// Sellisel juhul kasutatakse 16 bit asemel 32 (float)
    if (senseair_read(&dev, &measures.conc_ppm, &measures.temp_cC) == SENSEAIR_OK) {
//        printf("Concentration: %d ppm\n", measures.conc_ppm);
//        printf("Temperature: %4.2f °C\n", (measures.temp_cC / 100.));
    }

    if (senseair_read_abc_data(&dev, &abc_data) == SENSEAIR_OK) {
        puts("Saving sensor calibration data to FRAM.");
        if (fram_write(SENSEAIR_STATE_FRAM_ADDR, (uint8_t *)&abc_data, sizeof(abc_data))) {
            puts("FRAM write failed.");
        }
    }

    gpio_clear(ACME_BUS_POWER_PIN);
}

void voltage_read(void)
{
    gpio_init(VPANEL_ENABLE, GPIO_OUT);
    gpio_set(VPANEL_ENABLE);

    measures.vcc = adc_sample(ADC_VCC, ADC_RES_12BIT) * 4000 / 4095;  // corrected value (1V = 4095 counts)
   
    ztimer_sleep(ZTIMER_MSEC, 10);
    measures.vpanel = adc_sample(ADC_VPANEL, ADC_RES_12BIT) * 3933 / 4095; // adapted to real resistor partition value (75k over 220k)
//   puts("[SENSOR vpanel] READ.");

    gpio_clear(VPANEL_ENABLE);
}

void wakeup_task(void)
{
    puts("Wakeup task.");
    seaseair_sensor_read();
    voltage_read();
    printf("vcc: %ld, vpanel: %ld, CO2: %d, TEMP: %.1f\n",
                        measures.vcc, measures.vpanel,
                        measures.conc_ppm, (measures.temp_cC / 100.));
}

void periodic_task(void)
{
    puts("Periodic task.");
    hdc3020_sensor_read();     
    seaseair_sensor_read();
    voltage_read();
           
    printf("vcc: %ld, vpanel: %ld, temp: %.1f, hum: %.1f, CO2: %d, TEMP: %.1f\n",
                        measures.vcc, measures.vpanel,
                        measures.temp, measures.hum,
                        measures.conc_ppm, (measures.temp_cC / 100.));

}

/*
void boot_task(void)
{
    struct tm time;

    puts("Boot task.");
    rtc_localtime(0, &time);
    rtc_set_time(&time);
    fram_erase();
}
*/

int main(void)
{
    fram_init();
    
    switch (saml21_wakeup_cause()) {
    case BACKUP_EXTWAKE:
        wakeup_task();
        break;
    case BACKUP_RTC:
        periodic_task();
        break;
    default:
        printf("\n");
        printf("-------------------------------------\n");
        printf("-       PV-element katseseade       -\n");
        printf("-            by TalTech             -\n");
        printf("-  Version  : %s              -\n", FW_VERSION);
        printf("-  Compiled : %s %s  -\n", __DATE__, __TIME__);
        printf("-------------------------------------\n");
        printf("\n");

        if (extwake.pin != EXTWAKE_NONE) {
            printf("PUSH BUTTON P2 will wake the board.\n");
        }
        if (SLEEP_TIME > -1) {
            printf("Periodic task running every %d seconds.\n", SLEEP_TIME);
        }

//        boot_task();
        
        break;
    }

    puts("Entering backup mode.");
    saml21_backup_mode_enter(0, extwake, SLEEP_TIME, 1);
    // never reached
    return 0;
}
