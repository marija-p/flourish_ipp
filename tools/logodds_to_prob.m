function [prob] = logodds_to_prob(logodds)

prob = 1 - (1./(1+exp(logodds)));

end