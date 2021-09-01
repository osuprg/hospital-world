
import numpy as np
from statistics import mean
from matplotlib import pyplot as plt

with np.load('GMM-test-data-2021-08-31-100edges.npz') as data:
    my_data = data['my_data']
    their_data = data['their_data']

my_data_avg = []
their_data_avg = []

my_mean_per_edge = []
their_mean_per_edge = []

for i in range(len(my_data)):
    my_mean = mean(my_data[i])
    their_mean = mean(their_data[i])

    my_data_avg.append(my_mean)
    their_data_avg.append(their_mean)

    my_mean_per_edge.append(my_mean / (i + 2))
    their_mean_per_edge.append(their_mean / (i + 2))

# print(my_data_avg)
# print(their_data_avg)

x_vals = [i+2 for i in range(100)]

# plt.plot(x_vals, their_mean_per_edge, 'g', label='Convolution')
# plt.plot(x_vals, my_mean_per_edge, 'b', label='MCMC')
# plt.xlabel('Number of edges')
# plt.ylabel('Run time per edge (sec)')
# plt.legend()
# plt.show()

plt.clf()
plt.title('Time to calculate path cost distribution')
plt.plot(x_vals, their_data_avg, 'g', label='Convolution')
plt.plot(x_vals, my_data_avg, 'b', label='MCMC')
plt.xlabel('Number of edges')
plt.ylabel('Run time (sec)')
plt.legend()
plt.savefig('time-comparison-mcmc-convolution.png')

