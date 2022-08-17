A = 0
for i in 1:5
    B = 0
    if i == 1
        B = 1
    else
        A = A+B
    end
    println(A)
end
B = 0
i = 1
if i == 1
    B = 1
else
    A = A+B
end
println(B)
println(A)