function tilde6 = tilde6(T)
% Tilde for Twists (a 6x1 vector)
    w = T(1:3);
    v = T(4:6);
    tilde6 = [tilde3(w) v;
              0  0   0  0];
end