function [total_dist, segment_length] = calDistance(waypts, k_segment)
segment_length = [];
total_dist = 0;
for i=2:(k_segment+1)
    length = norm(waypts(:,i) - waypts(:,i-1));
    segment_length = [segment_length length];
    total_dist = total_dist + length;
end