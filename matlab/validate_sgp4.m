function result = validate_sgp4(traceFile, tleLine1, tleLine2, startTimeUtc)
% validate_sgp4  Compare ns-3 SatelliteOrbitMobilityModel positions vs MATLAB SGP4.
%
%   result = validate_sgp4(traceFile, tleLine1, tleLine2, startTimeUtc)
%
%   Inputs:
%     traceFile     - path to ntn-lena-nb-traces.txt produced by the ns-3 scenario
%     tleLine1      - TLE line 1 (69 chars) used in the ns-3 run
%     tleLine2      - TLE line 2 (69 chars) used in the ns-3 run
%     startTimeUtc  - UTC start time, e.g. '2026-04-24T12:00:00'
%                     (must match --simStartTimeUtc passed to ns-3)
%
%   Outputs (struct):
%     .t          - timestamps (seconds since sim start)
%     .ns3        - [lat lon alt_km] from ns-3 trace
%     .matlab     - [lat lon alt_km] from MATLAB SGP4
%     .errorKm    - 3D Euclidean position error per sample (km)
%     .maxErrorKm - worst-case error over the run
%     .meanErrorKm
%
%   Requires: Satellite Communications Toolbox (for satelliteScenario / satellite).

    arguments
        traceFile (1,:) char
        tleLine1  (1,:) char
        tleLine2  (1,:) char
        startTimeUtc (1,:) char
    end

    %% 1. Parse ns-3 trace file
    samples = parseNs3Trace(traceFile);
    if isempty(samples)
        error('No [SAT_POSITION] lines found in %s', traceFile);
    end
    fprintf('Parsed %d samples from %s\n', height(samples), traceFile);

    %% 2. Build MATLAB satellite scenario with identical TLE
    t0 = datetime(startTimeUtc, 'InputFormat','yyyy-MM-dd''T''HH:mm:ss', ...
                  'TimeZone','UTC');
    tEnd = t0 + seconds(max(samples.t));

    sc = satelliteScenario(t0, tEnd, 1);

    % Write the TLE to a temp file — satellite() needs a file path
    tmpTle = [tempname '.tle'];
    fid = fopen(tmpTle, 'w');
    fprintf(fid, 'SAT\n%s\n%s\n', tleLine1, tleLine2);
    fclose(fid);
    sat = satellite(sc, tmpTle);
    delete(tmpTle);

    %% 3. Query MATLAB positions at ns-3 timestamps
    matlabLatLonAlt = zeros(height(samples), 3);
    for i = 1:height(samples)
        t = t0 + seconds(samples.t(i));
        [pos, ~] = states(sat, t, 'CoordinateFrame','ecef');
        [lat, lon, h] = ecef2geodetic(wgs84Ellipsoid('meter'), ...
                                       pos(1), pos(2), pos(3));
        matlabLatLonAlt(i,:) = [lat, lon, h/1000];  % altitude in km
    end

    %% 4. Compute error in 3D ECEF
    wgs84 = wgs84Ellipsoid('meter');
    ns3Ecef = zeros(height(samples), 3);
    mlbEcef = zeros(height(samples), 3);
    for i = 1:height(samples)
        [x,y,z] = geodetic2ecef(wgs84, samples.lat(i), samples.lon(i), samples.altKm(i)*1000);
        ns3Ecef(i,:) = [x y z];
        [x,y,z] = geodetic2ecef(wgs84, matlabLatLonAlt(i,1), matlabLatLonAlt(i,2), matlabLatLonAlt(i,3)*1000);
        mlbEcef(i,:) = [x y z];
    end
    errorKm = vecnorm(ns3Ecef - mlbEcef, 2, 2) / 1000;

    %% 5. Report
    result = struct();
    result.t          = samples.t;
    result.ns3        = [samples.lat samples.lon samples.altKm];
    result.matlab     = matlabLatLonAlt;
    result.errorKm    = errorKm;
    result.maxErrorKm = max(errorKm);
    result.meanErrorKm = mean(errorKm);

    fprintf('\n=== SGP4 VALIDATION ===\n');
    fprintf('Samples:       %d\n', height(samples));
    fprintf('Max error:     %.3f km\n', result.maxErrorKm);
    fprintf('Mean error:    %.3f km\n', result.meanErrorKm);
    fprintf('First sample:  ns-3 (%.4f, %.4f, %.1fkm)  MATLAB (%.4f, %.4f, %.1fkm)\n', ...
        samples.lat(1), samples.lon(1), samples.altKm(1), ...
        matlabLatLonAlt(1,1), matlabLatLonAlt(1,2), matlabLatLonAlt(1,3));
    fprintf('Last sample:   ns-3 (%.4f, %.4f, %.1fkm)  MATLAB (%.4f, %.4f, %.1fkm)\n', ...
        samples.lat(end), samples.lon(end), samples.altKm(end), ...
        matlabLatLonAlt(end,1), matlabLatLonAlt(end,2), matlabLatLonAlt(end,3));

    % SGP4 is a deterministic algorithm — identical TLE and time should yield
    % sub-meter agreement. Anything above ~1 km indicates a bug (wrong time,
    % wrong coordinate frame, unit mismatch, etc.).
    if result.maxErrorKm < 1.0
        fprintf('\nResult: PASS (sub-km agreement — SGP4 implementations match)\n');
    elseif result.maxErrorKm < 10.0
        fprintf('\nResult: WARN (%.1f km error — check startTimeUtc and TLE)\n', result.maxErrorKm);
    else
        fprintf('\nResult: FAIL (%.1f km error — likely time-frame or coord-frame mismatch)\n', result.maxErrorKm);
    end

    %% 6. Plot
    figure('Name','SGP4 Validation');
    subplot(3,1,1);
    plot(samples.t, samples.lat, 'b-', samples.t, matlabLatLonAlt(:,1), 'r--');
    ylabel('Latitude (°)'); legend('ns-3','MATLAB','Location','best'); grid on;
    subplot(3,1,2);
    plot(samples.t, samples.lon, 'b-', samples.t, matlabLatLonAlt(:,2), 'r--');
    ylabel('Longitude (°)'); grid on;
    subplot(3,1,3);
    plot(samples.t, errorKm, 'k-');
    ylabel('3D error (km)'); xlabel('Sim time (s)'); grid on;
end


function T = parseNs3Trace(fname)
% Extract [SAT_POSITION] lines. Example line:
%   [SAT_POSITION] Time: 5s, sat0=(lat=12.1791, lon=118.1951, alt=574.8km) el=57.34°
    raw = fileread(fname);
    pat = '\[SAT_POSITION\]\s+Time:\s+([\d\.\-]+)s,\s+sat0=\(lat=([\-\d\.]+),\s+lon=([\-\d\.]+),\s+alt=([\d\.]+)km\)';
    tok = regexp(raw, pat, 'tokens');
    n = numel(tok);
    t     = zeros(n,1);
    lat   = zeros(n,1);
    lon   = zeros(n,1);
    altKm = zeros(n,1);
    for i = 1:n
        t(i)     = str2double(tok{i}{1});
        lat(i)   = str2double(tok{i}{2});
        lon(i)   = str2double(tok{i}{3});
        altKm(i) = str2double(tok{i}{4});
    end
    T = table(t, lat, lon, altKm);
end
