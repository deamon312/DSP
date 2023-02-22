function [yn,en,S] = LMSadapt(un,dn,S)
    mu = S.step;
    N = length(un); % number of samples un = dn
    yn = zeros(N,1); % initialize filter output vector (estimation y2')
    w = S.coeffs; % initialize filter coefficient vector
    en = zeros(N,1); % initialize error vector
    M = length(S.coeffs);
    S.W = zeros(M,N); % filter coefficient matrix for coeff. history
    for i = 1:N
      if i <= M % assume zero-samples for delayed data that isn't available
          k = i:-1:1;
          u = [un(k); zeros(M-numel(k),1)];
      else
          u = un(i:-1:i-M+1); % M samples of x in reverse order
      end
      yn(i) = w'*u; % filter output
      en(i) = dn(i) - yn(i); % error
      w = w + mu*en(i)'*u; % update filter coefficients
      S.W(:,i) = w; % store current filter coefficients in matrix
    end
    S.coeffs  = w ;
end