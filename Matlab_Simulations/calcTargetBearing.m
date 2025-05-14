function [target_bearing, status] = calcTargetBearing( ...
    curr_wp, prev_wp, curr_pos, vehicle_speed, ...
    lookahead_gain, lookahead_min, lookahead_max)

curr_wp = curr_wp(:);
prev_wp = prev_wp(:);
curr_pos = curr_pos(:);

% Validate inputs
if any(~isfinite([curr_wp(:); prev_wp(:); curr_pos(:); vehicle_speed]))
    target_bearing = NaN;
    status = struct(); return;
end

% Compute lookahead distance
lookahead_distance = max(min(lookahead_gain * abs(vehicle_speed), lookahead_max), lookahead_min);

% Vectors
vec_prev_to_curr_wp = curr_wp - prev_wp;
vec_prev_to_pos = curr_pos - prev_wp;
vec_pos_to_curr_wp = curr_wp - curr_pos;

if norm(vec_prev_to_curr_wp) < eps
    target_bearing = atan2(vec_pos_to_curr_wp(2), vec_pos_to_curr_wp(1));
    status = initStatus(); return;
end

unit_path = vec_prev_to_curr_wp / norm(vec_prev_to_curr_wp);
projection_len = dot(vec_prev_to_pos(:), unit_path(:));
pos_on_path = projection_len * unit_path;
vec_pos_to_path = pos_on_path - vec_prev_to_pos;

% Crosstrack error (signed)
crosstrack_error = sign(vec_prev_to_curr_wp(1) * vec_pos_to_path(2) - ...
                        vec_prev_to_curr_wp(2) * vec_pos_to_path(1)) * norm(vec_pos_to_path);

bearing_to_wp = wrapToPi(atan2(vec_pos_to_curr_wp(2), vec_pos_to_curr_wp(1)));

% Initialize default output
target_bearing = NaN;

if norm(vec_pos_to_curr_wp) < lookahead_distance || norm(vec_prev_to_curr_wp) < eps
    target_bearing = bearing_to_wp;

elseif abs(crosstrack_error) > lookahead_distance
    % No intersection — vehicle is too far off-path
    to_closest_prev = vec_pos_to_path + vec_prev_to_pos;
    to_closest_curr = vec_pos_to_path - vec_pos_to_curr_wp;

    if dot(to_closest_prev, vec_prev_to_curr_wp) < eps
        target_bearing = wrapToPi(atan2(-vec_prev_to_pos(2), -vec_prev_to_pos(1)));

    elseif dot(to_closest_curr, vec_prev_to_curr_wp) > eps
        target_bearing = bearing_to_wp;

    else
        target_bearing = wrapToPi(atan2(vec_pos_to_path(2), vec_pos_to_path(1)));
    end

else
    % Normal pursuit — calculate intersection point on the path
    extension = sqrt(lookahead_distance^2 - norm(vec_pos_to_path)^2);
    intersection = pos_on_path + extension * unit_path;
    vec_to_intersection = intersection - vec_prev_to_pos;
    target_bearing = wrapToPi(atan2(vec_to_intersection(2), vec_to_intersection(1)));
end

% Pack status
status = struct( ...
    'lookahead_distance', lookahead_distance, ...
    'target_bearing', target_bearing, ...
    'crosstrack_error', crosstrack_error, ...
    'distance_to_waypoint', norm(vec_pos_to_curr_wp), ...
    'bearing_to_waypoint', bearing_to_wp ...
);

end
