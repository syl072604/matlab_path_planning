function [distDiff, angleDiff]=stateDiffAB(stateA,stateB)
distDiff = stateB(1) - stateA(1);
angleDiff = stateB(2) - stateA(2);