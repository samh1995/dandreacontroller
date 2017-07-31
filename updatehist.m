function [Hist] = updatehist(Hist, t, state,f)
    % Initialize history of the state and its derivative
    Hist.states = [Hist.states, state];
    Hist.times = [Hist.times, t];
   Hist.f = [Hist.f, f];
       %Hist.n_desired = [Hist.n_desired, n_desired];
end