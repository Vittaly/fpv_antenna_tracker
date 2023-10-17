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

                    telemetryData.pitch = readInt(0);
                    telemetryData.roll = readInt(2);
                    telemetryData.heading = readInt(4);
                    break;

                case FrameCode::S:

                    telemetryData.voltage = readInt(0);
                    telemetryData.rssi = readByte(4);
                    raw = readByte(6);
                    telemetryData.flightMode = raw >> 2;
                    break;

                case FrameCode::G:

                    telemetryData.latitude = readInt32(0);
                    telemetryData.longitude = readInt32(4);
                    telemetryData.groundSpeed = readByte(8);
                    telemetryData.altitude = readInt32(9);

                    raw = readByte(13);
                    telemetryData.gpsSats = raw >> 2;
                    telemetryData.gpsFix = raw & 0x03;

                    gpsDataReceived = true;

                    break;

                case FrameCode::X:

                    telemetryData.hdop = readInt(0);
                    telemetryData.sensorStatus = readByte(2);
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

bool LTMParser::isGpsDataReceived()
{
    return gpsDataReceived;
}

remoteData_t LTMParser::getTelemetryData()
{
    return telemetryData;
}
