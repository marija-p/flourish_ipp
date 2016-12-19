function [logodds] = prob_to_logodds(prob)

logodds = log(prob./(1-prob));

end