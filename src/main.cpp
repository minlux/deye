//---------------------------------------------------------------------------------------------------------------------
/*!
   \file
   \brief Write and read Modbus TCP requests resp. response from DEYE micro inverter.

   Usage %s -a <ip-address -s <serial-number>

   <ip-address> specifies the IPv4 address of the inverter, as string, in dotted-decimal notation (eg. 192.168.178.139)
   <serial-number> is expected to be as decimal number (e.g. 3237994787)


   Note: To upload the data to a backend, you can pipe the output directly to curl:
   ```
   ./deye -a 192.168.178.139 -s 3942535924 | curl -H 'Content-Type: application-json' -X POST --data @- http://localhost:8080/pvmonitor/deye
   ```

   https://github.com/kbialek/deye-inverter-mqtt/blob/main/docs/metric_group_micro.md
   https://github.com/kbialek/deye-inverter-mqtt/blob/main/docs/metric_group_settings.md
   https://github.com/StephanJoubert/home_assistant_solarman/blob/main/custom_components/solarman/inverter_definitions/deye_2mppt.yaml
*/
//---------------------------------------------------------------------------------------------------------------------


/* -- Includes ------------------------------------------------------------ */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#include <fcntl.h>
#include <nlohmann/json.hpp>
#include "crc.h"



/* -- Defines ------------------------------------------------------------- */
// #define USE_DUMMY_RESPONSE

/* -- Types --------------------------------------------------------------- */

/* -- (Module-)Global Variables ------------------------------------------- */

#ifdef USE_DUMMY_RESPONSE
static const uint8_t response[] =
{
  0xa5, 0x87, 0x00, 0x10, 0x15, 0x00, 0x8d, 0xf4, 0x52, 0xfe, 0xea, 0x02,
  0x01, 0x8a, 0x57, 0x3e, 0x00, 0xb0, 0x28, 0x00, 0x00, 0xb7, 0xb7, 0x45,
  0x65, 0x01, 0x03, 0x74, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x08, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x04, 0x62, 0x00, 0x00, 0x03, 0xf6, 0x00, 0x00, 0x09, 0x2e, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0x88, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x3e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0e, 0x00, 0x05,
  0x01, 0x0f, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x3e, 0x45, 0xfb, 0x15
};
#endif



/* -- Function Prototypes ------------------------------------------------- */

/* -- Implementation ------------------------------------------------------ */
static void usage(const char* cmd)
{
   fprintf(stderr, "Usage %s -a <ip-address> -s <serial-number> [--req <file>] [--res <file>]\r\n\n", cmd);
}


static const char * secondsToHMS(char buffer[16], uint32_t seconds)
{   
   const uint32_t hours = (seconds / 3600);
   const uint32_t minutes = (seconds / 60) % 60;
   seconds = seconds % 60;
   sprintf(buffer, "%02u:%02u:%02u", hours, minutes, seconds);
   return buffer;
}

static const char * secondsToDHMS(char buffer[48], uint32_t seconds)
{   
   const uint32_t days = (seconds / 3600) / 24u;
   const uint32_t hours = (seconds / 3600) % 24;
   const uint32_t minutes = (seconds / 60) % 60;
   seconds = seconds % 60;
   sprintf(buffer, "%u Days, %u Hours, %u Minutes, %u Seconds", days, hours, minutes, seconds);
   return buffer;
}


class DeyeMicroInvModbusRegs
{
public:
   static constexpr unsigned int PV_MAX = 4; //up to 4 pv modules can be connected to the inverter
   uint32_t serialNumber;
   uint32_t totalOperatingSeconds;
   uint32_t dayOperatingSeconds;
   // uint8_t  rtcValue[4];
private:
   /* not implemented!!!
   uint8_t _invSerialNumber[5*2];   //@3..7; ASCII (eg. 32 33 30 35 30 36 37 37 32 34)
   uint8_t _hwVersion[2];           //@12; 0xMmRp (major.minor.release.patch, eg. 0x0102
   uint8_t _fwVersion[2];           //@13; 0xMmRp (major.minor.release.patch, eg. 0x0117)
   uint8_t _acVersion[2];           //@14; 0xMmRp (major.minor.release.patch, eg. 0x1218)
   uint8_t _ratedPower01W[2];       //@16; max. power of intverter (0x1F40)
   uint8_t _protocolVersion[2];     //@18; 0xMmRp (major.minor.release.patch, eg. 0x0201)
   uint8_t _powerLimitation[2];     //@40; limit power to x% max (eg. 0x0064)
   uint8_t _invSerialNumber[3*2];   //@22..24; 0xYYmm 0xDDHH 0xMMSS (YY/mm/DD HH:MM:SS, eg. 17 0c 15 09 28 08)
   */

   //Register 59 .. 116 -> 58 Registers
   //All registers are 16-bit
   //All data is big endian
   uint8_t _status[2];              //@59
   uint8_t _dayEnergy01kWh[2];      //@60
   uint8_t _res61[2];               //@61
   uint8_t _uptime1m[2];            //@62; minutes
   uint8_t _totalEnergy01kWh[4];    //@63,64
   uint8_t _pv1DayEnergy01kwh[2];   //@65
   uint8_t _pv2DayEnergy01kwh[2];   //@66
   uint8_t _pv3DayEnergy01kwh[2];   //@67
   uint8_t _pv4DayEnergy01kwh[2];   //@68
   uint8_t _pv1TotalEnergy01kwh[4]; //@69,70
   uint8_t _pv2TotalEnergy01kwh[4]; //@71,72
   uint8_t _acVoltage01V[2];        //@73
   uint8_t _pv3TotalEnergy01kwh[4]; //@74,75
   uint8_t _acCurrent01A[2];        //@76
   uint8_t _pv4TotalEnergy01kwh[4]; //@77,78
   uint8_t _acFrequency001Hz[2];    //@79
   uint8_t _operatingPower1W[2];    //@80
   uint8_t _res81_85[5][2];         //@81..85
   uint8_t _acPower1W[4];           //@86,87
   uint8_t _res88_89[2][2];         //@88..89
   uint8_t _radiatorTemperature001C[2]; //@90
   uint8_t _res91_108[18][2];       //@91..108
   uint8_t _pv1Voltage01V[2];       //@109
   uint8_t _pv1Current01A[2];       //@110
   uint8_t _pv2Voltage01V[2];       //@111
   uint8_t _pv2Current01A[2];       //@112
   uint8_t _pv3Voltage01V[2];       //@113
   uint8_t _pv3Current01A[2];       //@114
   uint8_t _pv4Voltage01V[2];       //@115
   uint8_t _pv4Current01A[2];       //@116

public:
   bool fromResponse(const uint8_t * data, uint32_t length)
   {
      //check for integrity
      //test for start byte
      if (data[0] != 0xA5) return false;
      //test for expected length
   #if 0 //todo
      if (length != 146)    return false; //given data lenght should match the expected number of bytes
      if (data[1] != 0x85)  return false; //i expect a frame length of 133 (0x0085) - which is the length of a frame with 57 register data
      if (data[2] != 0x00)  return false; //i expect a frame length of 133 (0x0085) - which is the length of a frame with 57 register data
      if (data[27] != 57*2) return false; //i expect modbus data for 57 registers
   #endif
      //verify sum byte
      uint8_t sum = 0;
      for (unsigned int i = 1; i < length-2; ++i)
      {
         sum += data[i];
      }
      if (sum != data[length-2]) return false;
      //verify modbus crc
      uint16_t crc = crc_calc_crc16_reflected(0xFFFF, data + 25, 3 + data[27] + 2);
      if (crc != 0) return false;
      //text for end byte
      if (data[length-1] != 0x15) return false;


      //on success
      // puts("valid response!");
      memset(this, 0, sizeof(*this));
      memcpy((void *)&serialNumber, data+7, 4);
      memcpy((void *)&totalOperatingSeconds, data+13, 4+4); //dayOperatingSeconds
      memcpy((void *)_status, data+28, data[27]);
      return true;
   }


   inline unsigned int status() //minutes
   {
      return _status[1]; //0: Stand-by; 1: Self-check; 2: Normal; 3: Warning; 4: Fault
   }

   inline unsigned int uptime() //minutes
   {
      return ((unsigned int)_uptime1m[0] << 8) | _uptime1m[1]; //get big endian value
   }

   inline double temperature() //Celsius
   {
      const unsigned int val = ((unsigned int)_radiatorTemperature001C[0] << 8) | _radiatorTemperature001C[1]; //get big endian value
      return 0.01 * val;
   }

   inline unsigned int operatingPower() //W
   {
      return ((unsigned int)_operatingPower1W[0] << 8) | _operatingPower1W[1]; //get big endian value
   }


   inline double acVoltage() //V
   {
      const unsigned int val = ((unsigned int)_acVoltage01V[0] << 8) | _acVoltage01V[1]; //get big endian value
      return 0.1 * val;
   }

   inline double acCurrent() //A
   {
      const unsigned int val = ((unsigned int)_acCurrent01A[0] << 8) | _acCurrent01A[1]; //get big endian value
      return 0.1 * val;
   }

   inline unsigned int acPower() //W
   {
      const unsigned int val = ((unsigned int)_acPower1W[0] << 8) | _acPower1W[1]; //get big endian value
      return 0.1 * val;
   }

   inline double acCalculatedPower() //W
   {
      return acVoltage() * acCurrent();
   }

   inline double acFrequency() //Hz
   {
      const unsigned int val = ((unsigned int)_acFrequency001Hz[0] << 8) | _acFrequency001Hz[1]; //get big endian value
      return 0.01 * val;
   }

   inline double acDayEnergy() //kWh
   {
      const unsigned int val = ((unsigned int)_dayEnergy01kWh[0] << 8) | _dayEnergy01kWh[1]; //get big endian value
      return 0.1 * val;
   }

   inline double acTotalEnergy() //kWh
   {
      const unsigned int val =
         ((unsigned int)_totalEnergy01kWh[2] << 24) |
         ((unsigned int)_totalEnergy01kWh[3] << 16) |
         ((unsigned int)_totalEnergy01kWh[0] << 8) |
         _totalEnergy01kWh[1]; //get big endian value
      return 0.1 * val;
   }


   inline double pvDayEnergy(unsigned int pv) //kWh
   {
      const uint8_t * const d = _pv1DayEnergy01kwh + 2*pv;
      const unsigned int val = ((unsigned int)d[0] << 8) | d[1]; //get big endian value
      return 0.1 * val;
   }

   inline double pvTotalEnergy(unsigned int pv) //kWh
   {
      const uint8_t offset[4] = { 0, 4, 10, 16 };
      const uint8_t * const d = _pv1TotalEnergy01kwh + offset[pv];
      const unsigned int val =
         ((unsigned int)d[2] << 24) |
         ((unsigned int)d[3] << 16) |
         ((unsigned int)d[0] << 8) |
         d[1]; //get big endian value
      return 0.1 * val;
   }

   inline double pvVoltage(unsigned int pv) //V
   {
      const uint8_t * const d = _pv1Voltage01V + 4*pv;
      const unsigned int val = ((unsigned int)d[0] << 8) | d[1]; //get big endian value
      return 0.1 * val;
   }

   inline double pvCurrent(unsigned int pv) //A
   {
      const uint8_t * const d = _pv1Current01A + 4*pv;
      const unsigned int val = ((unsigned int)d[0] << 8) | d[1]; //get big endian value
      return 0.1 * val;
   }

   inline double pvPower(unsigned int pv) //W
   {
      return pvVoltage(pv) * pvCurrent(pv);
   }

   nlohmann::json json()
   {
      nlohmann::json result;
      char tmp[48];

      result["sn"] = serialNumber;
      result["status"] = status();
      result["uptime"] = secondsToHMS(tmp, dayOperatingSeconds);
      result["total_optime"] = {
         { "seconds", totalOperatingSeconds                     },
         { "dhms",    secondsToDHMS(tmp, totalOperatingSeconds) }
      };
      // result["rtc"] = hexdump(tmp, rtcValue, sizeof(rtcValue));
      // result["operating_power"] = operatingPower();
      result["radiator_temperature"] = temperature();

      //PV1..4
      result["pv"] = nlohmann::json::array();
      for (unsigned int i = 0; i < PV_MAX; ++i)
      {
         result["pv"].push_back({
            { "voltage",      pvVoltage(i)     },
            { "current",      pvCurrent(i)     },
            { "power",        pvPower(i)       },
            { "day_energy",   pvDayEnergy(i)   },
            { "total_energy", pvTotalEnergy(i) }
         });
      }

      //AC
      result["ac"] = {
         { "voltage",      acVoltage()     },
         { "current",      acCurrent()     },
         { "power",        acPower()       },
         { "frequency",    acFrequency()   },
         { "day_energy",   acDayEnergy()   },
         { "total_energy", acTotalEnergy() }
      };

      return result;
   }
};





int main(int argc, char * argv[])
{
#ifndef USE_DUMMY_RESPONSE
   //evaluate command line arguments
   //-i <ip-address> (default: NULL)
   char const* ipAddress = NULL;
   for (int i = 1; i < (argc - 1); ++i)
   {
      if ((argv[i][0] == '-') && (argv[i][1] == 'a'))
      {
         ipAddress = argv[i + 1];
         break;
      }
   }

   //-s <serial-number (default: 0)
   unsigned int serialNumber = 0;
   for (int i = 1; i < (argc - 1); ++i)
   {
      if ((argv[i][0] == '-') && (argv[i][1] == 's'))
      {
         serialNumber = (unsigned int)atol(argv[i + 1]);
         break;
      }
   }


   //--req <file> (default: NULL)
   char const* reqFile = NULL;
   for (int i = 1; i < (argc - 1); ++i)
   {
      if (strcmp(argv[i], "--req") == 0)
      {
         reqFile = argv[i + 1];
         break;
      }
   }

   //--res <file> (default: NULL)
   char const* resFile = NULL;
   for (int i = 1; i < (argc - 1); ++i)
   {
      if (strcmp(argv[i], "--res") == 0)
      {
         resFile = argv[i + 1];
         break;
      }
   }



   //check for mandatory arguments
   if (!ipAddress || !serialNumber)
   {
      fprintf(stderr, "IP address and serial number of inverter expected!\n");
      usage(argv[0]);
      return -1;
   }


   //try to create socket
   int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
   if (sock < 0)
   {
      fprintf(stderr, "Failed to open socket!\n");
      return -2;
   }
   //configure send/receive timeouts
   {
      constexpr unsigned int sendRecvTimeout1s = 5;
      struct timeval sendTimeout = { 0 };
      struct timeval recvTimeout = { 0 };
      sendTimeout.tv_sec = sendRecvTimeout1s;
      recvTimeout.tv_sec = sendRecvTimeout1s;
      setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&sendTimeout, sizeof(struct timeval));
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvTimeout, sizeof(struct timeval));
   }


   //send request to inverter
   {
      uint8_t requestTemplate[36] =
      {
         0xa5, //start byte
         0x17, 0x00, //length (13 + modbus-frame-length + 2); little endian
         0x10, 0x45, //control code
         0x00, 0x00, //???
         0x00, 0x00, 0x00, 0x00, //@7: serial number; little endian
         0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //???
         //@26: start of modbus frame
            0x01, //unit id
            0x03, //function code (read holding registers)
            0x00, 0x3b, //first register address (59 - big endian)
            0x00, 0x3a, //number of registers to read (58 - big endian)
            0xb4, 0x14, //modbus crc of modbus frame (little endian)
         //@34: end of modbus frame
         0x00, //checksum (low byte of sum of all precedint bytes, starting after the start-byte (0x17+0x00+...0x45+0xd4))
         0x15  //end byte
      };

      //set serial number
      memcpy(&requestTemplate[7], (const uint8_t *)&serialNumber, 4);

      //calculate modbus checksum
      {
         uint16_t crc = crc_calc_crc16_reflected(0xFFFF, &requestTemplate[26], 6);
         requestTemplate[32] = (uint8_t)crc;
         requestTemplate[33] = (uint8_t)(crc >> 8);
      }

      //calculate checksum
      uint8_t sum = 0;
      for (unsigned int i = 1; i < 34; ++i)
      {
         sum += requestTemplate[i];
      }
      requestTemplate[34] = sum;

      //optional: dump request to file
      if (reqFile)
      {
         FILE *f = fopen(reqFile, "wb");
         if (f)
         {
            fwrite(requestTemplate, 1, sizeof(requestTemplate), f);
            fclose(f);
         }
         else
         {
            fprintf(stderr, "[Warning] Failed to dump request to file %s!\n", reqFile);
         }
      }


      //connect to inverter
      {
         struct sockaddr_in address;  /* the libc network address data structure */
         constexpr uint16_t port = 8899;
         uint32_t addr = inet_addr(ipAddress); //convert string representation of IP address (decimals and dots) to binary
         address.sin_addr.s_addr = addr;  /* assign the address */
         address.sin_port = htons(port);  /* translate int2port num */
         address.sin_family = AF_INET;
         int status = connect(sock, (struct sockaddr *)&address, sizeof(address));
         if (status != 0)
         {
            fprintf(stderr, "Failed to open connect to %s!\n", ipAddress);
            close(sock);
            return -3;
         }
      }
      //send to inverter
      int status = send(sock, requestTemplate, sizeof(requestTemplate), 0);
      if (status < sizeof(requestTemplate))
      {
         fprintf(stderr, "Failed to send request!\n");
         close(sock);
         return -4;
      }
   }


   //receive response
   uint8_t buffer[4096]; //should be enough
   int status = recv(sock, buffer, sizeof(buffer), 0);
   close(sock);
   if (status <= 0) //on success
   {
      fprintf(stderr, "Failed to receive response. Error %d!\n", status);
      return -5;
   }


   //optional: dump response to file
   if (resFile)
   {
      FILE *f = fopen(resFile, "wb");
      if (f)
      {
         fwrite(buffer, 1, status, f);
         fclose(f);
      }
      else
      {
         fprintf(stderr, "[Warning] Failed to dump response to file %s!\n", resFile);
      }
   }
#else
   const uint8_t * const buffer = response;
   const uint32_t status = sizeof(response);
#endif

   //decode response
   DeyeMicroInvModbusRegs inverter;
   if (inverter.fromResponse(buffer, status) == false)
   {
      fprintf(stderr, "Failed to process response data!\n");
      return -6;
   }


   //output decoded data as json
   puts(inverter.json().dump().c_str());
   return 0;
}

