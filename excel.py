import openpyxl as xl

wb = xl.load_workbook('Parametros.xlsx')
sheet = wb['Sheet1']
class Parametros:
    def __init__(self, line):
        self.line = line
        sla = []
        for col in range(2, 5):
            sla.append(sheet.cell(line, col).value)
        self.value = sla

list_names = ['w', 'Tc', 'Pc','Zc', 'Vc', 'A', 'B', 'C', 'Vi', 'Zi']
j = 1
new = 0
list_parameters = []
for i in list_names:
    j += 1
    #list_parameters[i] = Parametros(j).value
    list_parameters.append(Parametros(j).value)
    new += 1

print(list_parameters)

for i in range(len(list_parameters[0])):
    for j in range(i+1, len(list_parameters[0])):
        wij = (list_parameters[0][i] + list_parameters[0][j])/2
        print(i+1, " e ", j+1, ": " ,wij)





    #def make_list (**i):


