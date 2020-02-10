function rand_num = genRandomNumber(a, b, N)
    if ~exist("N", "var")
        N = 1;
    end
    rand_num = a + (b-a) .* rand(N, 1);
end