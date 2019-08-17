function xs = sat(x,L)
if x <= -L; xs = -L; end
if x >= L; xs = L; end
if x < L & x > -L; xs = x; end
