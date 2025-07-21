/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "lsm6dsrsensor.h"

#include "GlobalVars.h"
#include "utils.h"
#include "helper_3dmath.h"
#include "magneto1.4.h"
#include "calibration.h"

#include <i2cscan.h>

void LSM6DSR_Sensor::motionSetup(){
    //Initialize I2C bus.
    imu.initialize(addr);
    if (!imu.testConnection()) {
        m_logger.fatal(
            "Can't connect to %d at address 0x%%02x", Address);
            ledManager.pattern(50, 50, 200);
            return;    
        
    }
    imu->Enable_X();
    imu->Enable_G();

    if (!hallsensor.testConnection()){
        m_logger.fatal(
            "Can't connect to mag");
            ledManager.pattern(100,25,200);
            return;
        
    }

}

void LSM6DSR_Sensor::startCalibration(int calibrationType){
    constexpr  int  calibrationSamples = 300;
    ledManager.on();

    //calibration code for accelerometer
    MagnetoCalibration *magneto =  new MagnetoCalibration;
    for(int i = 0; i <= calibrationSamples; i++){
        int16_t acc[3];
        imu->Get_G_AxesRaw(acc);
        magneto->sample(acc[0],acc[1],acc[2]);
        delay(200); 
    }
    m_Logger.debug("Calculating calibration data...");

    float A_BAinv[4][3];
    magneto->current_calibration(A_BAinv);
    //

    m_Logger.debug("[INFO] Accelerometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.Ac[i] = A_BAinv[0][i];
		m_Config.Ac_Cinv[0][i] = A_BAinv[1][i];
		m_Config.Ac_Cinv[1][i] = A_BAinv[2][i];
		m_Config.Ac_Cinv[2][i] = A_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			A_BAinv[0][i],
			A_BAinv[1][i],
			A_BAinv[2][i],
			A_BAinv[3][i]
		);
	}
	m_Logger.debug("}");
    ledManager.pattern(500,100,10);

    //calibration code for magnetometer 
    for(int  i = 0;  i  <= calibrationSamples; i++) {
        ledManager.on();
        int16_t mx, my, mz;
        hallSensor.readData(mx, my, mz);
        magneto->sample(mx,my,mz);

        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float M_BAinv[4][3];
    magneto->current_calibration(M_BAinv);
// 

    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
	m_Logger.debug("{");
	for (int i = 0; i < 3; i++) {
		m_Config.Mg[i] = M_BAinv[0][i];
		m_Config.Mg_Cinv[0][i] = M_BAinv[1][i];
		m_Config.Mg_Cinv[1][i] = M_BAinv[2][i];
		m_Config.Mg_Cinv[2][i] = M_BAinv[3][i];
		m_Logger.debug(
			"  %f, %f, %f, %f",
			M_BAinv[0][i],
			M_BAinv[1][i],
			M_BAinv[2][i],
			M_BAinv[3][i]
		);
	}
	m_Logger.debug("}");

    m_Logger.debug("Calibration has been completed");
	m_tpsCounter.reset();
	m_dataCounter.reset();
    // 0.01 s sampling time, i.e. 100 Hz
    VQF vqf_object(0.01);
    vqf = &vqf_object;
    vqf->setMagRef(norm,dip);
    motionLoop();
}

void LSM6DSR_Sensor::motionLoop(){

    //calibrated accelerometer readings
    int16_t a[3];
    imu->Get_X_AxesRaw(a);
    for(int i=0; i<3; i++){
        acc[i]*=9.78023;
    //this g value is calculated for the given region and can vary slightly depending on location
    }
    for(int i = 0; i < 3; i++){
        acc[i] = m_Config.Ac_Cinv[i][0]*(a[0]-m_Config.Ac[0])
               + m_Config.Ac_Cinv[i][1]*(a[1]-m_Config.Ac[1])
               + m_Config.Ac_Cinv[i][2]*(a[2]-m_Config.Ac[2]);
    }
    acceleration.x = acc[0];
    acceleration.y = acc[1];
    acceleration.z = acc[2];

    //calibrated magnetometer readings
    int16_t m[3];
    hallSensor.readData(m[0],m[1],m[2]);
    for(int i = 0; i < 3; i++){
        mag[i] = m_Config.Mg_Cinv[i][0]*(a[0]-m_Config.Mg[0])
               + m_Config.Mg_Cinv[i][1]*(a[1]-m_Config.Mg[1])
               + m_Config.Mg_Cinv[i][2]*(a[2]-m_Config.Mg[2]);
    }

    //gyroscope readings(not calibrated, expecting vqf to handle it)
    int32_t gy_temp[3];
    imu->Get_G_Axes(gy_temp);

    //vqf for sensor fusion 
    gy_temp[0]=gyro[0];
    gy_temp[1]=gyro[1];
    gy_temp[2]=gyro[2];
    vqf->update(acc,gyro,mag);
    vqf->getQuat9D(fusedValue);
    fusedRotation.x = fusedValue[0];
    fusedRotation.y = fusedValue[1];
    fusedRotation.z = fusedValue[2];
    fusedRotation.w = fusedValue[3];
    //newFusedRotation = true;
}

void LSM6DSR_Sensor::sendData() {
    if (newFusedRotation){
        newFusedRotation = false;
        networkConnection.sendRotationData(
        sensorId, &fusedRotation, DATA_TYPE_NORMAL,
        0);

#ifdef DEBUG_SENSOR
		m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif

#if SEND_ACCELERATION
    if (newAcceleration) {
        newAcceleration = false;
        networkConnection.sendSensorAcceleration(
            this->sensorId,
            this->acceleration
        );
    }
#endif
}
}