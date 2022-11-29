function decimate(rawSampleRate) %#codegen

decimationFactor = 48;
fprintf('Channelizer: Starting up...\n')
coder.varsize('state')

%Channelization Settings
maxNumChannels      = 256;
nChannels           = decimationFactor; %Decimation is currently set to equal nChannels. Must be a factor of rawFrameLength
pauseWhenIdleTime   = 0.25;

%UDP Settings
udpReceivePort      = 10000;
udpCommandPort      = 10001;
udpServePorts       = 20000:20000+maxNumChannels-1;%10000:10039;

%Incoming Data Variables
rawFrameLength  = 128;

rawFrameTime        = rawFrameLength/rawSampleRate;
bytesPerSample      = 8;
supportedSampleRates = [912 768 456 384 256 192]*1000;

if ~any(rawSampleRate == supportedSampleRates)
    error(['UAV-RT: Unsupported sample rate requested. Available rates are [',num2str(supportedSampleRates/1000),'] kS/s.'])
end


fprintf('Channelizer: Setting up output channel UDP ports...\n')
samplesPerChannelMessage = 1024; % Must be a multiple of 128
samplesAtFlush           = samplesPerChannelMessage * decimationFactor;
bytesPerChannelMessage   = bytesPerSample * samplesPerChannelMessage+1;%Adding 1 for the time stamp items on the front of each message. 
sendBufferSize           = 2^nextpow2(bytesPerChannelMessage);
dataBufferFIFO           = dsp.AsyncBuffer(2*samplesAtFlush);
write(dataBufferFIFO,single(1+1i));%Write a single value so the number of channels is specified for coder. Specify complex single for airspy data
read(dataBufferFIFO);     %Read out that single sample to empty the buffer.

udpReceive = dsp.UDPReceiver('RemoteIPAddress',         '127.0.0.1',    ...
                                'LocalIPPort',          10000,          ...
                                'ReceiveBufferSize',    2^18,           ...
                                'MaximumMessageLength', 4096,           ...
                                'MessageDataType',      'single',       ...
                                'IsMessageComplex',     true);
udpSend = dsp.UDPSender('RemoteIPAddress',         '127.0.0.1',      ...
                                'RemoteIPPort',         20000,          ...
                                'SendBufferSize',       sendBufferSize)''
totalSampsReceived = 0;
frameIndex = 1;

expectedFrameSize = rawFrameLength;
bufferTimeStamp4Sending = complex(single(0));
while true
    dataReceived = udpReceive();
    if (~isempty(dataReceived))               
        if frameIndex == 1
            bufferTimeStamp = round(10^3*posixtime(datetime('now')));
            bufferTimeStamp4Sending = int2singlecomplex(bufferTimeStamp);
        end
        sampsReceived = numel(dataReceived);
        totalSampsReceived = totalSampsReceived + sampsReceived;
        %Used to keep a running estimated of the expected frame
        %size to help identifiy subsize frames received. 
        if sampsReceived<expectedFrameSize
            disp('Subpacket received')
        end
        if sampsReceived~=expectedFrameSize
            expectedFrameSize = round(mean([sampsReceived, expectedFrameSize]));
        end
        write(dataBufferFIFO,dataReceived(:));%Call with (:) to help coder realize it is a single channel
        
        frameIndex = frameIndex+1;

        if dataBufferFIFO.NumUnreadSamples>=samplesAtFlush
            fprintf('Channelizer: Running - Buffer filled with %u samples. Flushing to channels. Currently receiving: %i samples per packet.\n',uint32(samplesAtFlush),int32(expectedFrameSize))
            fprintf('Actual time between buffer flushes: %6.6f.  Expected: %6.6f. \n', toc, samplesAtFlush/rawSampleRate)
            frameIndex = 1;
            tic;
            y = stageddecimate(double(read(dataBufferFIFO, samplesAtFlush)), rawSampleRate, 4000);
        	data = [bufferTimeStamp4Sending; y];
            udpSend(data)
            time2Channelize = toc;
            fprintf('Time required to channelize: %6.6f \n', time2Channelize)
        end
    else
        pause(rawFrameTime/2);
    end
end