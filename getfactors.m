function [facts] = getfactors(x) %#codegen
%GETFACTORS Gets the factors of a number passed to it similar to
%divisors.m. 
%   This function returns all the integer factors of an number passed to
%   it. This function was written because divisors.m did not support
%   code generation. If x is negative, the factors of the absolute values
%   are returned. 
%
%INPUTS:
%   x   A numeric for which the factors are needed
%
%OUTPUTS:
%   facts A vector of the factors of the absolute value of x
%
%
%Author: Michael W. Shafer
%Date:   20201-08-02
%
%%
if numel(x)~=1
    error('UAV-RT: Input must be a single numeric value')
end

if ~isreal(x)
    error('UAV-RT: Input must real valued')
end

x = abs(x);
    
facts = zeros(1,ceil(x/2));
i = 2;
tick = 0;
while i <= floor(x/2)
    if mod(x,i)==0 && x>1
        tick = tick+1;
        facts(tick) = i;
    end
    i = i+1;
end

facts(facts==0)=[];
if x~=1
    facts = [1 facts x];
else
    facts = 1;
end


end

