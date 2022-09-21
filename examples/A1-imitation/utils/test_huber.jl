using LinearAlgebra

A = [1.0 2.0 4 8]
B = [3.0 3 3 3]
abs_diff = abs.(A - B)
α = 1

mask = abs_diff .> 1
huber = abs_diff .* mask
quadratic = abs_diff .* .!mask

abs2.(A - B)

J = 0.5 * quadratic * Diagonal(ones(4)) * transpose(quadratic)
J += huber* ones(4)*α 

transpose(quadratic)
quadratic
Diagonal(ones(4))

α.*huber
size(mask)
huber* ones(4)*α
α^2
abs_diff[mask]
sum(abs_diff[mask])
sum(abs_diff[.!mask])
abs2.(abs_diff[mask])