import time
import random
def quase_ordenado(array_ordenado, perturbacao):
    tamanho = len(array_ordenado)
    elementos_a_embaralhar = int(tamanho * perturbacao)

    for _ in range(elementos_a_embaralhar):
        idx_a, idx_b = random.randint(0, tamanho - 1), random.randint(0, tamanho - 1)
        array_ordenado[idx_a], array_ordenado[idx_b] = array_ordenado[idx_b], array_ordenado[idx_a]

    return array_ordenado

inicio = time.time()

n = 5000000  # Número de zeros desejados

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(5000000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(5000000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

random.shuffle(cores)

cores.sort()
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(cores)

#///////////////////////////////////////////////////////////////////////////#

inicio = time.time()

n = 5000000  # Número de zeros desejados

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(5000000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(5000000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

cores = quase_ordenado(cores, 0.1)  # Perturbação de 10%

cores.sort()
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(cores)

#//////////////////////////////////////////////////////////#

inicio = time.time()

n = 500000  # Número de zeros desejados

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(7250000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(7250000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

random.shuffle(cores)

cores.sort()
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(cores)

#//////////////////////////////////////////////////////////////////////////

inicio = time.time()

n = 500000  # Número de zeros desejados

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(7250000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(7250000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

cores = quase_ordenado(cores, 0.1)  # Perturbação de 10%

cores.sort()
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(cores)

#/////////////////////////////////////////////////////////////////////////////

inicio = time.time()

def dutch_flag_partition(nums):
    low, mid, high = 0, 0, len(nums) - 1

    while mid <= high:
        if nums[mid] < 0:  # Se for negativo, coloca no início
            nums[low], nums[mid] = nums[mid], nums[low]
            low += 1
            mid += 1
        elif nums[mid] == 0:  # Se for zero, mantém no meio
            mid += 1
        else:  # Se for positivo, coloca no final
            nums[mid], nums[high] = nums[high], nums[mid]
            high -= 1

    return nums

n = 5000000

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(5000000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(5000000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

random.shuffle(cores)

resultado = dutch_flag_partition(cores)
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(resultado)

#///////////////////////////////////////////////////////////////////////

inicio = time.time()

def dutch_flag_partition(nums):
    low, mid, high = 0, 0, len(nums) - 1

    while mid <= high:
        if nums[mid] < 0:  # Se for negativo, coloca no início
            nums[low], nums[mid] = nums[mid], nums[low]
            low += 1
            mid += 1
        elif nums[mid] == 0:  # Se for zero, mantém no meio
            mid += 1
        else:  # Se for positivo, coloca no final
            nums[mid], nums[high] = nums[high], nums[mid]
            high -= 1

    return nums

n = 5000000

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(5000000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(5000000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

cores = quase_ordenado(cores, 0.1)  # Perturbação de 10%

resultado = dutch_flag_partition(cores)
fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(resultado)

#///////////////////////////////////////////////////////////////////////////

inicio = time.time()

def dutch_flag_partition(nums):
    low, mid, high = 0, 0, len(nums) - 1

    while mid <= high:
        if nums[mid] < 0:  # Se for negativo, coloca no início
            nums[low], nums[mid] = nums[mid], nums[low]
            low += 1
            mid += 1
        elif nums[mid] == 0:  # Se for zero, mantém no meio
            mid += 1
        else:  # Se for positivo, coloca no final
            nums[mid], nums[high] = nums[high], nums[mid]
            high -= 1

    return nums

n = 500000

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(7250000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(7250000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

random.shuffle(cores)
resultado = dutch_flag_partition(cores)

fim = time.time()

# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')
#print(resultado)

#//////////////////////////////////////////////////////////////////////

inicio = time.time()

def dutch_flag_partition(nums):
    low, mid, high = 0, 0, len(nums) - 1

    while mid <= high:
        if nums[mid] < 0:  # Se for negativo, coloca no início
            nums[low], nums[mid] = nums[mid], nums[low]
            low += 1
            mid += 1
        elif nums[mid] == 0:  # Se for zero, mantém no meio
            mid += 1
        else:  # Se for positivo, coloca no final
            nums[mid], nums[high] = nums[high], nums[mid]
            high -= 1

    return nums

n = 500000

zeros = [0] * n

negativos = [round(random.uniform(-10000, 0), 2) for _ in range(7250000)]

positivos = [round(random.uniform(0, 10000), 2) for _ in range(7250000)]

# negativos: vermelho, 0's: branco , positivos: azul
cores = negativos + zeros + positivos

cores = quase_ordenado(cores, 0.1)  # Perturbação de 10%
resultado = dutch_flag_partition(cores)

fim = time.time()
#print(resultado)
# Calcula o tempo decorrido
tempo_decorrido = fim - inicio
print(f'Tempo decorrido: {tempo_decorrido} segundos')