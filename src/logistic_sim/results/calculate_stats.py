import os
import csv
import math

#  import numpy <--- agevola il lavoro
#  import matplot se vogliamo fare grafici con i dati 

def calculate_means(sums, n_exps):
    result = []
    for t in sums:
        result.append( t/n_exps )
    return result

def calculate_sqms(data, means, n_exps, n_robots):
    result = []
    temp = [0] * n_robots
    for exp_data in data:
        for robot in range(n_robots):
            temp[robot] += (int(exp_data[robot]) - means[robot])**2
    for robot in range(n_robots):
        result.append(math.sqrt(temp[robot] / max(1, n_exps-1)))
    return result

def main():
    for subdir, dirs, files in os.walk('.'):
        if subdir != '.':
            experiments = len(files)
            robots = 0

            # conto numero di robots dal numero di righe
            with open(os.path.join(subdir, files[0])) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for row in csv_reader:
                    robots = robots + 1
                robots = robots - 1

            print robots
            distances = []
            tot_distances = [0.0]*robots
            mean_distances = []
            sqm_distances = []

            interferences = []
            tot_interfs = [0.0]*robots
            mean_interfs = []
            sqm_interfs = []

            times = []
            tot_times = [0.0]*robots
            mean_times = []
            sqm_times = []

            missions = []
            tot_missions = [0.0]*robots
            mean_missions = []
            sqm_missions = []

            tasks = []
            tot_tasks = [0.0]*robots
            mean_tasks = []
            sqm_tasks = []

            # raccolta dati
            for f in files:
                path = os.path.join(subdir, f)
                with open(path) as csv_file:
                    csv_reader = csv.reader(csv_file, delimiter=',')
                    line_count = 0
                    d = []
                    i = []
                    t = []
                    m = []
                    ts = []
                    for row in csv_reader:
                        if line_count != 0:
                            d.append(row[1])
                            tot_distances[line_count-1] += int(row[1])
                            i.append(row[2])
                            tot_interfs[line_count-1] += int(row[2])
                            m.append(row[3])
                            tot_missions[line_count-1] += int(row[3])
                            ts.append(row[4])
                            tot_tasks[line_count-1] += int(row[4])
                            t.append(row[5])
                            tot_times[line_count-1] += int(row[5])
                        line_count = line_count + 1
                    distances.append(d)
                    interferences.append(i)
                    missions.append(m)
                    tasks.append(ts)
                    times.append(t)

            # calcolo medie
            mean_distances = calculate_means(tot_distances, experiments)
            mean_interfs = calculate_means(tot_interfs, experiments)
            mean_missions = calculate_means(tot_missions, experiments)
            mean_tasks = calculate_means(tot_tasks, experiments)
            mean_times = calculate_means(tot_times, experiments)

            # calcolo sqm
            sqm_distances = calculate_sqms(distances, mean_distances, experiments, robots)
            sqm_interfs = calculate_sqms(interferences, mean_interfs, experiments, robots)
            sqm_missions = calculate_sqms(missions, mean_missions, experiments, robots)
            sqm_tasks = calculate_sqms(tasks, mean_tasks, experiments, robots)
            sqm_times = calculate_sqms(times, mean_times, experiments, robots)
            
            path = os.path.join(subdir, 'aggregated.csv')
            with open(path, mode="w") as csv_file:
                fieldnames = ['id_robot','dist_mean','dist_sqm','interf_mean','interf_sqm','missions_mean','missions_sqm','tasks_mean','tasks_sqm','times_mean','times_sqm']
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

                csv_writer.writeheader()
                for robot in range(robots):
                    csv_writer.writerow({'id_robot':robot,
                                            'dist_mean':mean_distances[robot], 'dist_sqm':sqm_distances[robot],
                                            'interf_mean':mean_interfs[robot], 'interf_sqm':sqm_interfs[robot],
                                            'missions_mean':mean_missions[robot], 'missions_sqm':sqm_missions[robot],
                                            'tasks_mean':mean_tasks[robot], 'tasks_sqm':sqm_tasks[robot],
                                            'times_mean':mean_times[robot], 'times_sqm':sqm_times[robot]
                                        })
            # print distances, tot_distances, mean_distances, sqm_distances
            # print interferences, tot_interfs, mean_interfs, sqm_interfs
            # print missions, tot_missions, mean_missions, sqm_missions
            # print tasks, tot_tasks, mean_tasks, sqm_tasks
            # print times, tot_times, mean_times, sqm_times
            # print ''
                    

if __name__ == "__main__":
    main()