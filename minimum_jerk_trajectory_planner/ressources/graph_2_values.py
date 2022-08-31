import matplotlib.pyplot as plt
  
values1 = []
values2 = []
t_values = []

file_name = input("File name: ")
name1 = input("Variable1 name: ")
name2 = input("Variable2 name: ")
  
f_values = open(file_name,'r')
for row_values in f_values:
    if not row_values:
        continue
    row_values = row_values.split(' ')
    values2.append(float(row_values[2]))
    values1.append(float(row_values[1]))
    t_values.append(float(row_values[0]))

plt.figure(1)
plt.plot(t_values, values1, label=name1)  
plt.plot(t_values, values2, label=name2)  
plt.title(file_name, fontsize = 20)
plt.legend()
plt.show()