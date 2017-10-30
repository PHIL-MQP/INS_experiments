#include "WPILib.h"
#include "AHRS.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

/**
 * This is a demo program providing a real-time display of navX
 * MXP values.
 *
 * In the operatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX MXP sensor.
 *
 */
class Robot: public IterativeRobot
{
	std::shared_ptr<NetworkTable> table;
    AHRS *ahrs;
    LiveWindow *lw;
    int autoLoopCounter;

public:
    Robot() :
        table(NULL),
        ahrs(NULL),
        lw(NULL),
        autoLoopCounter(0)
    {
    }

private:
    void RobotInit()
    {
        table = NetworkTable::GetTable("datatable");
        lw = LiveWindow::GetInstance();
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI::Port::kMXP);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs ) {
            LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
        }
	}

    void AutonomousInit()
    {
        autoLoopCounter = 0;
        collectSamples(6000);

    }

    void collectSamples(int b)
    {
    	long long elapsed = 0;
    	int c = 0;
    	//int b = 5000; //50 seconds of data
    	struct timeval t1;
    	struct timeval t0;
    	typedef struct ins {
    		float imu[3];
    		float gyro[3];
    		struct timeval timestamp;
    	} INS;


    	// collect c samples at 100 Hz
		printf("start stream\n.\n.\n.\n");
		gettimeofday(&t0, 0);
		gettimeofday(&t1, 0);
		INS temp;
		INS buffer[b];
		while(c < b){
			gettimeofday(&t1, 0); //in microseconds (
					elapsed = (t1.tv_sec-t0.tv_sec)*1000000 + t1.tv_usec-t0.tv_usec;
					//100 samples per second (Hz)
					if(elapsed > 10000){
						//printf("%f %f %f %f %f %f %ld\n",ahrs->GetRawGyroX(), ahrs->GetRawGyroY(), ahrs->GetRawGyroZ(), ahrs->GetRawAccelX(), ahrs->GetRawAccelY(), ahrs->GetRawAccelZ(), elapsed);
						temp.imu[0] = ahrs->GetRawAccelX();
						temp.imu[1] = ahrs->GetRawAccelY();
						temp.imu[2] = ahrs->GetRawAccelZ();
						temp.gyro[0] = ahrs->GetRawGyroX();
						temp.gyro[1] = ahrs->GetRawGyroY();
						temp.gyro[2] = ahrs->GetRawGyroZ();
						gettimeofday(&temp.timestamp, 0);
						memcpy(&buffer[c], &temp, sizeof temp);
						gettimeofday(&t0, 0);
						gettimeofday(&t1, 0);
						c++;
					}
		}
		FILE * pFile;
		pFile = fopen ("ins_data.txt","w");
		if (pFile!=NULL)
		{
			for(int i = 0; i < b; i++) {
				fprintf(pFile
					, "%f %f %f %f %f %f %ld %ld\n"
					, buffer[i].imu[0]
					, buffer[i].imu[1]
					, buffer[i].imu[2]
					, buffer[i].gyro[0]
					, buffer[i].gyro[1]
					, buffer[i].gyro[2]
					, buffer[i].timestamp.tv_sec
					, buffer[i].timestamp.tv_usec);
			}
		}
		fclose(pFile);

		printf("%d samples collected\n", c);
    }

    void AutonomousPeriodic()
    {

    }

    void TeleopInit()
    {
    }

    void TeleopPeriodic()
    {
        if ( !ahrs ) return;

        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */

        SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
        SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
        SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
        SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
        SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
        SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
        SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
        SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
        SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());

    }

    void TestPeriodic()
    {
        lw->Run();
    }
};

START_ROBOT_CLASS(Robot);
