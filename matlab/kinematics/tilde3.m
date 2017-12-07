function tilde3 = tilde3(w)
% Tilde operator for a 3x1 vector
    tilde3 =     [0     -w(3)   w(2);
                  w(3)   0     -w(1);
                 -w(2)   w(1)   0  ];
end
