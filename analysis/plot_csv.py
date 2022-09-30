# -*- coding: utf-8 -*-
"""
Created on Tue Sep 27 17:10:10 2022

@author: Rick
"""
import csv
import matplotlib.pyplot as plt
import itertools

if __name__ == "__main__":
    with open("28-09-2022.csv") as csv_file:
        csv_reader = csv.reader(csv_file)
        
        csv_data = []
        csv_settings = {}
        for line in csv_reader:
            if len(line) > 1:
                csv_data.append(line)
            else:
                try:
                    key, value = line[0].split("=")
                    csv_settings[key] = float(value)
                except IndexError:
                    pass
        
        header = csv_data[0][2:]
        num_data = [[float(v) for v in row[2:-1]] for row in csv_data[1:]]
        n_x = len(num_data)
        num_data = zip(*num_data)
            
        (fig, axs) = plt.subplots(3, 2, constrained_layout=True, sharex=True)
        i_ax = [0, 1, 2, 2, 3, 4, 2, 2, 2, 5]
        axs = list(itertools.chain(*axs))
        
        xs = [x * csv_settings["TIMESTEP_UPDATE"] / 3600 for x in range(0, n_x)]
        
        for i, row in enumerate(num_data):
            ax = axs[i_ax[i]]
            #ax.set_title(header[i])
            ax.grid(True)
            ax.plot(xs, row, "--o", ms=4, label=header[i], markerfacecolor="None")
            ax.legend()
            

        axs[3].axhline(csv_settings["slope_max_per_sample"], ls='--', c='r')
        axs[5].axhline(csv_settings["REAL_MIN_DUTYCYCLE"] * 255, ls='--', c='r')
        axs[5].axhline(csv_settings["REAL_MAX_DUTYCYCLE"] * 255, ls='--', c='r')