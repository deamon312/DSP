function S = LMSinit(w0,mu)
    S.coeffs = w0(:);
    S.step = mu;
