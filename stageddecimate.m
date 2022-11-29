function [x] = stageddecimate(xin,Fsraw,Fsdec) %#codegen
%STAGEDDECIMATE Decimates the input vector in stages in accordance
%with the limit suggested by decimate.m, where factors greater than 13
%should be decimated in stages.
%   The Matlab function decimate.m states that if the decimation factor is
%   greater than 13, the decimation factor should be divided and done in
%   stages. This function does this staged decimation. If reviews the
%   requested decimation factor and proceeds to decimate using factors of
%   the decimation, attempting at each stages to decimate by the largest
%   factor to avoid unesseary decimation steps:
%
%INPUTS: 
%   xin     A column (Nx1) vector of data to be decimated. Maximum length is
%           10e6. If a matrix is entered, each column will be independently
%           decimated. 
%   Fsraw   Sample rate of x input
%   Fsdec   Desired sample rate of output
%   Fsdec   Desired sample rate of output
%
%OUTPUTS
%   x   The decimated vector of input as requested sample rate
%
%
%
%
%Author: Michael W. Shafer
%Date:   2021-08-02
%
%%
%check that Fsraw<=Fsdec
if Fsraw<Fsdec
    error('UAV-RT: Requested decimated sample rate must be less than or equal to raw sample rate.')
end

%check that Fsraw and Fsdec are positive
%check that Fsraw<=Fsdec
if Fsraw<0 && Fsdec<0
    error('UAV-RT: Sample frequency must be greater than zero.')
end

%check that Fsdec is a factor of Fsraw
if mod(Fsraw,Fsdec)~=0
    error('UAV-RT: Decimated sample rate must be an factor the raw sample rate.')
end


inputdims = size(xin);
if numel(inputdims)>2
   error('UAV-RT: Input must be a vector')
end
max_input_length = 1000e6;
if numel(xin)>max_input_length
   error('UAV-RT: Maxiumum supported input length is 10e6')
end

%x = xin; %Force column vector;
%Let coder know x will change size. Needed for end of for loop during
%decimation. We can't just do this on xin because it is an input variable
%with preset type. 
%coder.varsize('x',numel(xin));
coder.varsize('x',[1 max_input_length],[0 1]);
coder.varsize('xfilt_hi',[1 max_input_length],[0 1]);
x = 0+1i*0;%Initialize
xfilt_hi = 0+1i*0;%Initialize


currdecfact = Fsraw/Fsdec;

maxnumfactors = numel(factor(currdecfact));
divisorlist = zeros(1,maxnumfactors);%Preallocate
%check the decfactor is not a prime number

decfactlimit = 13;%Limit specifed by the decimate.m function

%Run through a loop that always takes the highest factor to decimate by.
%Keep decimating until no factors left. 
totalDecimation = 1;
i = 1;
while currdecfact~=1
    thedivisors = getfactors(currdecfact);%Had to build our own divisors.m function that could be deployed with code generator
    validdivisors_inds = thedivisors<=decfactlimit;
    validdivisors = thedivisors(validdivisors_inds);
    currdivisor = validdivisors(end);
    totalDecimation = totalDecimation * currdivisor
    divisorlist(i) = currdivisor;
    currdecfact = currdecfact/currdivisor;
    i = i+1;
end
%Get rid of excess entries.
divisorlist(divisorlist==0)=[];

fprintf("Final sample rate: %f %d\n", Fsraw / totalDecimation, totalDecimation)

filtord = 4;
b = zeros(1,filtord+1);%Preallocate for codegen
a = zeros(1,filtord+1);
%figure;
%plot((0:numel(xin)-1)/Fsraw,real(xin));hold on
for i = 1:numel(divisorlist)
    %Use a order 4 not 8 as specified by decimate.m since we use filtfilt
    %which results in a filtering twice that of the order specified by b
    %and a. See the filtfilt help for more information. The cheby1 call
    %would normally be called like this:
    %[b,a] = cheby1(4,0.05,0.8*1/divisorlist(i));
    %But for coder use, the inputs must be a constant, per the
    %documentation. To get around this, we create an if statement below,
    %each with their own normalized cutoff frequency corresponding to
    %0.8*1/divisorlist(i)
    % >> 0.8*1./1:13
    % 0.8 0.4 0.2667 0.2 0.16 0.133 0.1143 0.1 0.889 0.08 0.0727 0.667
    % 0.0615
    if divisorlist(i)==1
        [b,a] = cheby1(4,0.05,0.8);
    elseif divisorlist(i)==2
        [b,a] = cheby1(4,0.05,0.4);
    elseif divisorlist(i)==3
        [b,a] = cheby1(4,0.05,0.2667);
    elseif divisorlist(i)==4
        [b,a] = cheby1(4,0.05,0.2);
    elseif divisorlist(i)==5
        [b,a] = cheby1(4,0.05,0.16);
    elseif divisorlist(i)==6
        [b,a] = cheby1(4,0.05,0.1333);
    elseif divisorlist(i)==7
        [b,a] = cheby1(4,0.05,0.1143);
    elseif divisorlist(i)==8
        [b,a] = cheby1(4,0.05,0.1);
    elseif divisorlist(i)==9
        [b,a] = cheby1(4,0.05,0.0889);
    elseif divisorlist(i)==10
        [b,a] = cheby1(4,0.05,0.08);
    elseif divisorlist(i)==11
        [b,a] = cheby1(4,0.05,0.0727);
    elseif divisorlist(i)==12
        [b,a] = cheby1(4,0.05,0.0667);
    elseif divisorlist(i)==13
        [b,a] = cheby1(4,0.05,0.0615);
    end
    if i==1
        xfilt_hi = filtfilt(b,a,xin);
    else
        xfilt_hi = filtfilt(b,a,x);
    end
    
    %if i==1;currFs = Fsraw;else;currFs = Fsraw/prod(divisorlist(1:i-1));end
    %plot((0:numel(xfilt_hi)-1)/currFs,real(xfilt_hi));hold on
    x = downsample(xfilt_hi,divisorlist(i));
    %currFs = Fsraw./prod(divisorlist(1:i));
    %plot((0:numel(x)-1)/currFs,real(x));hold on
end



end

