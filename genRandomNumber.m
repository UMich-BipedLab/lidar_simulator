function rand_num = genRandomNumber(a, b, seed, N)
    if exist('seed', 'var')
        rng(seed)
    else
        rng(rand())
    end
    if ~exist("N", "var")
        N = 1;
    end
    rand_num = a + (b-a) .* rand(N, 1);
end