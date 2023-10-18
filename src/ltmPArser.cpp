#include <ltmParser.h>

byte LTMParser::readByte(uint8_t offset)
{
    return serialBuffer[offset];
}

int LTMParser::readInt(uint8_t offset)
{
    return (int)serialBuffer[offset] + ((int)serialBuffer[offset + 1] << 8);
}

int32_t LTMParser::readInt32(uint8_t offset)
{
    return (int32_t)serialBuffer[offset] + ((int32_t)serialBuffer[offset + 1] << 8) + ((int32_t)serialBuffer[offset + 2] << 16) + ((int32_t)serialBuffer[offset + 3] << 24);
}

bool LTMParser::parseChar(char data)
{

    bool res = false;

    if (state == IDLE)
    {
        if (data == '$')
            state = HEADER_START1;
    }
    else if (state == HEADER_START1)
    {
        if (data == 'T')
        {
            state = HEADER_START2;
        }
        else
        {
            state = IDLE;
        }
    }
    else if (state == HEADER_START2)
    {
        // frameProperties = FRAME_TYPE_UNKNOWN;
        state = HEADER_MSGTYPE;
        receiverIndex = 0;
        checkSumCalc = 0;
        // check the frame code from serial (data) for value of available codes (FrameTypeList)

        frameProperties = *std::find_if(std::begin(FrameTypeList), std::end(FrameTypeList), [data](const FrameProperties &item)
                                        { return item.code == data; });

        if (frameProperties.code == FRAME_TYPE_UNKNOWN.code)
            state = IDLE;
    }
    else if (state == HEADER_MSGTYPE)
    {

        /*
         * Check if last payload byte has been received.
         */
        if (receiverIndex == frameProperties.length)
        {
            res = true; // complete receive frame
            /*
             * If YES, check checksum and execute data processing
             */
            if (checkSumCalc != data)
            {
                lastFrameCheckSumErr = true;
                SerialUSB.println("incorrect checksum");
            }
            else
            {
                uint8_t raw;

                switch (frameProperties.code)
                {
                case FrameCode::A:

                    telemetryData.attitudeInfo.pitch = readInt(0);
                    telemetryData.attitudeInfo.roll = readInt(2);
                    telemetryData.attitudeInfo.heading = readInt(4);
                    
                    attitudeDataUpdated = false;
                    break;

                case FrameCode::S:

                    telemetryData.statusInfo.vbat = readInt(0);
                    telemetryData.statusInfo.rssi = readByte(4);
                    raw = readByte(6);
                    telemetryData.statusInfo.flightMode = raw >> 2;
                    
                    statusDataUpdated = true;
                    break;

                case FrameCode::G:

                    telemetryData.gpsInfo.latitude = readInt32(0);
                    telemetryData.gpsInfo.longitude = readInt32(4);
                    telemetryData.gpsInfo.groundSpeed = readByte(8);
                    telemetryData.gpsInfo.altitude = readInt32(9);

                    raw = readByte(13);
                    telemetryData.gpsInfo.gpsSats = raw >> 2;
                    telemetryData.gpsInfo.gpsFix = (GPSFix)(raw & 0x03);

                    gpsDataUpdated = true;

                    break;

                case FrameCode::X:

                    telemetryData.hdop = readInt(0);
                    telemetryData.sensorStatus = readByte(2);
                    extendDataUpdated = true;

                    break;
                }

                state = IDLE;
                // memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);
            }
        }
        else
        {
            /*
             * If no, put data into buffer
             */
            serialBuffer[receiverIndex++] = data;
            checkSumCalc ^= data;
        }
    }
    return res;
}

bool LTMParser::isGpsDataUpdated()
{           
    return gpsDataUpdated;
}

RemoteData_s LTMParser::getTelemetryData()
{
    gpsDataUpdated = false;
    attitudeDataUpdated = false;
    statusDataUpdated = false;
    extendDataUpdated = false;
    
    return telemetryData;
    
}

GPSFrameData_s& LTMParser::getGPSData()
{
    gpsDataUpdated = false;
    return telemetryData.gpsInfo;
}
