% Example: validate an ns-3 run against MATLAB SGP4.
%
% Prerequisite: run the ns-3 scenario once with a known start time, e.g.
%   ./ns3 run "lena-nb-5G-ntn-scenario --simTime=30s \
%              --simName=sateliot-test \
%              --simStartTimeUtc=2026-04-24T12:00:00"
%
% Then point this script at the trace file and re-use identical TLE + time.

% -- Edit these three values to match your ns-3 run --
traceFile    = '../logs/sateliot-test-1200km-seed21/20260424_155351/ntn-lena-nb-traces.txt';
startTimeUtc = '2026-04-24T12:00:00';
tleLine1     = '1 60550U 24149CL  26114.18615092  .00001182  00000+0  10549-3 0  9995';
tleLine2     = '2 60550  97.6763 190.9896 0005148 253.9991 106.0664 14.97828949 91985';

result = validate_sgp4(traceFile, tleLine1, tleLine2, startTimeUtc);
