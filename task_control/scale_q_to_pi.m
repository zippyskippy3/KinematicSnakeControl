function q_wrapped = scale_q_to_pi(q)
    % Wrap angles in q to the range [-pi, pi]
    q_wrapped = wrapToPi(q);

end
