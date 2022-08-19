import matplotlib.pyplot as plt
  
values = []
t_values = []

name = input("Variable name(file name without .txt): ")
  
f_values = open(name+'.txt','r')
for row_values in f_values:
    if not row_values:
        continue
    row_values = row_values.split(' ')
    values.append(float(row_values[1]))
    t_values.append(float(row_values[0]))

print(sum(values)/float(len(values)))

plt.figure(1)
plt.plot(t_values, values)  
plt.title(name, fontsize = 20)
plt.legend()
plt.show()