/**
 * @file MC_AS5600.c
 * @brief AS5600 magnetic encoder driver and rotor angle alignment/estimation for FOC.
 * @author reisen_fil (reisen_oxj@qq.com)
 * @version 1.0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026 
 * 
 */

#include "MC_AS5600.h"

/**
 * @brief Resolves the circular wrap-around of the encoder delta to calculate the true shortest positional difference.
 * @param delta The raw difference between two consecutive encoder readings.
 * @return The resolved delta value, adjusted for the encoder's circular resolution (ENCODER_RESOLUTION).
 * @date 2026-06-19
 */
static int16_t EncoderAlign_ResolveDelta(int16_t delta)
{
    const int16_t halfResolution = ENCODER_RESOLUTION / 2;
    
    if (delta > halfResolution)
    {
        delta -= ENCODER_RESOLUTION;
    }
    else if (delta < -halfResolution)
    {
        delta += ENCODER_RESOLUTION;
    }

    return delta;
}

/**
  * @brief  Read the raw 12-bit mechanical angle from the AS5600 sensor through I2C
  * @param  encoder_pHandle: Pointer to the encoder parameter structure used to store the read result
  * @retval uint8_t  I2C communication status (0: success, non-zero: failure)
  * @date 2026-06-19
  */
uint8_t AS5600_GetAngle(ENCODER_PARAM_T *encoder_pHandle)
{
    uint8_t AS5600_data[2];

    // TIM2->CNT = 0;    
    // Directly get the return value from the low-level I2C read function
    uint8_t i2c_status = drv_SoftI2C_ReadTwoBytes(AS5600_I2C_ADDR, AS5600_REG_ANGLE_H,AS5600_data);

    // AS5600_Get_cnt = TIM2->CNT; 

    if (!i2c_status) // Read successful
    {
        // Combine into a 12-bit angle: lower 4 bits of the high byte + 8 bits of the low byte
        uint16_t angle = ((AS5600_data[0] & 0x0F) << 8) | AS5600_data[1];
        // Reverse the direction and directly store it into the passed structure member
        encoder_pHandle->rawEncoder = 4095 - angle;
    }

    return i2c_status; // Return the I2C read status for upper-layer judgment
}


/* Parameter initialization */

/**
 * @brief Initializes the encoder parameter structure and resets all angle and state variables to zero.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @date 2026-06-02
 */
void EncoderAlign_Init(ENCODER_PARAM_T *encoder_pHandle)
{
    encoder_pHandle->ElectricalAngle_Norm = _IQ(0.0f);
    encoder_pHandle->MechanicalAngle_Norm = _IQ(0.0f);
    encoder_pHandle->Norm_speed = _IQ(0.0f);
    encoder_pHandle->rawEncoder = 0;
    encoder_pHandle->acc_rawEncoder = 0;          
    encoder_pHandle->calib_rawEncoder = 0;           
    encoder_pHandle->nowEncoder = 0;
    encoder_pHandle->lastEncoder = 0;
    encoder_pHandle->isCalibrated = 0;
    encoder_pHandle->alignState = 0;      
}

/**
 * @brief Executes the encoder zero-offset calibration sequence by injecting a d-axis current to align the rotor.
 * 
 * Calibration Principle:
 * 1. Apply positive d-axis voltage to pull the rotor to the positive d-axis direction.
 * 2. Read the encoder value at this position as the zero-offset.
 * 3. Subsequent electrical angle = (mechanical angle - zero-offset) * pole pairs.
 *
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @param  foc_phandle     Pointer to the main FOC system structure.
 * @date 2026-06-02
 */
void EncoderAlign_Calibrate(ENCODER_PARAM_T *encoder_pHandle, MC_FOC_SYSTEM_T *foc_phandle)
{
    // static uint32_t encoderSum;
    
    // State 1: Apply d-axis current and wait for rotor to stabilize
    if(encoder_pHandle->alignState == 0)
    {
        // Set d-axis current to alignment current, q-axis to 0
        FOC_Ctrl(_IQ(0.0), MC_Ud_Calib, foc_phandle);   // Uq=0, Ud=0.4
        encoder_pHandle->encoder_state_cnt++;
        if(encoder_pHandle->encoder_state_cnt >= 6000)
        {
            encoder_pHandle->encoder_state_cnt = 0;
            encoder_pHandle->alignState = 1;                
        }
    }

    // State 2: Sample encoder value 10 times and calculate the average
    if(encoder_pHandle->alignState == 1) 
    {
        if(encoder_pHandle->encoder_state_cnt < 10) 
        {
            // First determine whether the AS5600 read was successful
            if (!AS5600_GetAngle(encoder_pHandle)) 
            {
                // Read successful: accumulate the angle value stored in the structure into acc_rawEncoder
                encoder_pHandle->acc_rawEncoder += encoder_pHandle->rawEncoder;
                encoder_pHandle->encoder_state_cnt++;
            }
            else 
            {
                FOC_Ctrl(_IQ(0.0f), _IQ(0.0f), foc_phandle);

                // Read failed: clear the counter and alignment status flag, then trigger a system fault
                encoder_pHandle->encoder_state_cnt = 0;
                encoder_pHandle->alignState = 0;
                encoder_pHandle->acc_rawEncoder = 0; // Clear the accumulator for later use                
                
                encoder_pHandle->calib_error_cnt++;
                if(encoder_pHandle->calib_error_cnt <= 3) 
                {                      
                    foc_phandle->state_machine.current_state = FOC_STATE_FAULT;  /* Detection becomes invalid if the signal line contact becomes unstable or disconnects midway during I2C bus detection */
                }
                else foc_phandle->state_machine.current_state = FOC_STATE_STOP;
            }
        }
        else 
        {
            // Accumulation complete; calculate the average and enter the next stage
            encoder_pHandle->calib_rawEncoder = encoder_pHandle->acc_rawEncoder / encoder_pHandle->encoder_state_cnt;
            encoder_pHandle->calib_error_cnt = 0;
            encoder_pHandle->encoder_state_cnt = 0;
            encoder_pHandle->alignState = 2;
            encoder_pHandle->acc_rawEncoder = 0; // Clear the accumulator for later use
        }
    }    
    
    // State 3: Calculate normalized zero-offset
    if(encoder_pHandle->alignState == 2)
    {
        encoder_pHandle->lastEncoder = encoder_pHandle->calib_rawEncoder;
        encoder_pHandle->Norm_speed = _IQ(0.0f);
        
        // Stop current output
        FOC_Ctrl(_IQ(0.0f), _IQ(0.0f), foc_phandle);
        
        // Calibration completed
        encoder_pHandle->isCalibrated = 1;
        encoder_pHandle->alignState = 0;
    }
}

/**
 * @brief Updates the normalized mechanical and electrical angles based on the raw encoder reading and pole pairs.
 * 
 * Calculation Formula:
 * 1. Mechanical Angle (normalized) = (encoderRaw - encoderOffset) / ENCODER_RESOLUTION
 * 2. Electrical Angle (normalized) = Mechanical Angle * Pole Pairs
 * 3. Electrical Angle (normalized) = fmod(Electrical Angle, 1.0) // Ensure 0.0-1.0 range
 *
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @date 2026-06-02
 */
void EncoderAlign_UpdateAngle(ENCODER_PARAM_T *encoder_pHandle,MC_FOC_SYSTEM_T *foc_phandle)
{
    _iq mechAngleRaw;      // Raw mechanical angle (Q15)
    _iq mechAngleNorm;     // Normalized mechanical angle (Q15, 0.0-1.0)
    _iq elecAngleRaw;      // Raw electrical angle (Q15)
    _iq elecAngleNorm;     // Normalized electrical angle (Q15, 0.0-1.0)
    
    // Check if calibration is done
    if(encoder_pHandle->isCalibrated == 0)
    {
        encoder_pHandle->ElectricalAngle_Norm = _IQ(0.0f);
        return;
    }

    // Read real-time mechanical angle from encoder
    if(!AS5600_GetAngle(encoder_pHandle)) 
    {
        encoder_pHandle->nowEncoder = encoder_pHandle->rawEncoder;
        
        // Step 1: Calculate raw mechanical angle (considering zero-offset)
        // Handle encoder wrap-around (0-4095 cycle)
        if(encoder_pHandle->nowEncoder >= encoder_pHandle->calib_rawEncoder)
        {
            mechAngleRaw = encoder_pHandle->nowEncoder - encoder_pHandle->calib_rawEncoder;
        }
        else
        {
            // Handle wrap-around: encoder jumps from 4095 to 0
            mechAngleRaw = encoder_pHandle->nowEncoder + ENCODER_RESOLUTION - encoder_pHandle->calib_rawEncoder;
        }
        
        // Step 2: Normalize mechanical angle to 0.0-1.0 range
        mechAngleNorm = _IQdiv(_IQ(mechAngleRaw), _IQ(ENCODER_RESOLUTION));
        encoder_pHandle->MechanicalAngle_Norm = mechAngleNorm;
        
        // Step 3: Calculate electrical angle = mechanical angle * pole pairs
        elecAngleRaw = _IQmpy(mechAngleNorm, _IQ(MOTOR_POLE_PAIRS));
        
        // Step 4: Normalize electrical angle to 0.0-1.0 range (extract fractional part)
        // Since there are 7 pole pairs, electrical angle can be 0-7, need fmod(x, 1.0)
        elecAngleNorm = _IQfrac(elecAngleRaw);  // Extract fractional part -> normalized
        
        encoder_pHandle->ElectricalAngle_Norm = elecAngleNorm;        
    }
    else foc_phandle->state_machine.current_state = FOC_STATE_STOP;
}

/**
 * @brief Updates the estimated mechanical speed of the encoder based on the position difference and time interval.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @param  dt_us           Speed update period in microseconds.
 * @date 2026-06-02
 */
void EncoderAlign_UpdateSpeed(ENCODER_PARAM_T *encoder_pHandle, uint16_t dt_us)
{
    if (encoder_pHandle->isCalibrated == 0 || dt_us == 0)
    {
        encoder_pHandle->Norm_speed = _IQ(0.0f);
        return;
    }
    
    int16_t deltaRaw = encoder_pHandle->nowEncoder - encoder_pHandle->lastEncoder;
    deltaRaw = EncoderAlign_ResolveDelta(deltaRaw);
    
    encoder_pHandle->motorDir = (deltaRaw >= 0) ? 1 : 0;    /* 1: Forward rotation, 0: Reverse rotation */
    
    _iq Norm_deltaRaw = NORM_ENCODER(deltaRaw);
    
    // Prevent dt_us from being too small
    if (dt_us < 10) dt_us = 10;
    
    // Calculation: Norm_speed = (Norm_deltaRaw / dt_us) * 120000
    // Step 1: Normalized speed per microsecond
    _iq speed_per_us = _IQmpyI32(Norm_deltaRaw, 1000000);
    
    // Step 2: Convert to RPM per-unit value (use _IQmpyI32 to avoid overflow)
    _iq result = _IQdiv(speed_per_us, _IQ(dt_us));
    
    encoder_pHandle->Norm_speed = result;        /* 0 -> 1 */
    encoder_pHandle->lastEncoder = encoder_pHandle->nowEncoder;
}

/**
 * @brief Retrieves the current estimated mechanical speed of the motor.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @return _iq Mechanical speed in Q15 format (unit: rev/s).
 * @date 2026-06-02
 */
_iq EncoderAlign_GetSpeed(ENCODER_PARAM_T *encoder_pHandle)
{
    return encoder_pHandle->Norm_speed;
}

/**
 * @brief Retrieves the current normalized electrical angle (0.0 to 1.0) for FOC calculations.
 * 
 * Usage Notes:
 * - Returns a Q15 fixed-point number.
 * - 0.0 corresponds to 0° electrical angle.
 * - 0.25 corresponds to 90° electrical angle.
 * - 0.5 corresponds to 180° electrical angle.
 * - 0.75 corresponds to 270° electrical angle.
 * - 1.0 corresponds to 360° electrical angle (equivalent to 0).
 *
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @return _iq Normalized electrical angle in Q15 format (0.0-1.0).
 * @date 2026-06-02
 */
_iq EncoderAlign_GetElectricalAngle(ENCODER_PARAM_T *encoder_pHandle)
{
    return encoder_pHandle->ElectricalAngle_Norm;
}

/**
 * @brief Retrieves the current normalized mechanical angle (0.0 to 1.0) for debugging or speed estimation.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @return _iq Normalized mechanical angle in Q15 format (0.0-1.0).
 * @date 2026-06-02
 */
_iq EncoderAlign_GetMechanicalAngle(ENCODER_PARAM_T *encoder_pHandle)
{
    return encoder_pHandle->MechanicalAngle_Norm;
}

/**
 * @brief Retrieves the latest raw 12-bit encoder reading.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @return _iq The latest raw encoder value.
 * @date 2026-06-02
 */
_iq EncoderAlign_GetNowEncoder(ENCODER_PARAM_T *encoder_pHandle)
{
    return encoder_pHandle->nowEncoder;
}

/**
 * @brief Checks if the encoder zero-offset calibration has been successfully completed.
 * @param  encoder_pHandle Pointer to the encoder parameter structure.
 * @return uint8_t 1 if calibrated, 0 if not calibrated.
 * @date 2026-06-02
 */
uint8_t EncoderAlign_IsCalibrated(ENCODER_PARAM_T *encoder_pHandle)
{
    return encoder_pHandle->isCalibrated;
}

/**
 * @brief Converts a normalized angle (0.0 to 1.0) to radians (0.0 to 2*PI) for observers or trigonometric functions.
 * 
 * Conversion Formula: Radians = Normalized Angle * 2 * PI
 *
 * @param  normAngle Normalized angle in Q15 format (0.0-1.0).
 * @return _iq Angle in radians (Q15 format).
 * @date 2026-06-02
 */
_iq EncoderAlign_NormToRad(_iq normAngle)
{
    return _IQmpy(normAngle, _IQ(6.283185307f));  // 2 * PI
}

