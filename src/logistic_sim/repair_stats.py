import sys
import os
import statistics

def read_repair_stats_file(filename):
    data = {}
    with open(filename, "r") as f:
        lines = f.readlines()
    
    data['filename'] = filename
    data['obstacle_events'] = int(lines[0].split(":")[1])
    data['successfull_sa_repair'] = int(lines[1].split(":")[1])
    data['successfull_ma_repair'] = int(lines[2].split(":")[1])
    s = lines[4].split(":")[1]
    durations = []
    for d in s.split():
        durations.append(float(d))
    data['durations'] = durations
    s = lines[5].split(":")[1]
    attempts = []
    for a in s.split():
        attempts.append(int(a))
    data['attempts'] = attempts
    
    return data

def avg_data(data):
    print(data['filename'])
    new_data = data.copy()
    
    if len(data['attempts']) > 0:
        new_data['avg_attempts'] = statistics.mean(data['attempts'])
        new_data['sum_attempts'] = sum(data['attempts'])
    if len(data['durations']) > 0:
        new_data['avg_durations'] = statistics.mean(data['durations'])
        new_data['sum_durations'] = sum(data['durations'])
    return new_data

def aggregate_runs(dir):
    runs = []
    aggregate = {}
    for entry in os.scandir(dir):
        if(entry.path.endswith(".txt") and entry.is_file()):
            runs.append(avg_data(read_repair_stats_file(entry.path)))

    attempts_sum = 0
    durations_sum = 0
    avg_attempts = []
    avg_durations = []
    ma_repairs = []
    sa_repairs = []
    sum_attempts = []
    sum_durations = []
    for r in runs:
        if 'avg_attempts' in r:
            attempts_sum = attempts_sum + r['avg_attempts']
            avg_attempts.append(r['avg_attempts'])
        if 'avg_durations' in r:
            durations_sum = durations_sum + r['avg_durations']
            avg_durations.append(r['avg_durations'])
        ma_repairs.append(r['successfull_ma_repair'])
        sa_repairs.append(r['successfull_sa_repair'])
        if 'sum_attempts' in r:
            sum_attempts.append(r['sum_attempts'])
        if 'sum_durations' in r:
            sum_durations.append(r['sum_durations'])
    aggregate['avgavg_attempts'] = attempts_sum / len(runs)
    aggregate['avgavg_durations'] = durations_sum / len(runs)
    aggregate['stdavg_attempts'] = statistics.stdev(avg_attempts)
    aggregate['stdavg_durations'] = statistics.stdev(avg_durations)
    aggregate['avg_ma_repairs'] = statistics.mean(ma_repairs)
    aggregate['avg_sa_reapirs'] = statistics.mean(sa_repairs)
    aggregate['avgsum_attempts'] = statistics.mean(sum_attempts)
    aggregate['avgsum_durations'] = statistics.mean(sum_durations)

    return aggregate

a = aggregate_runs(sys.argv[1])
print('avgsum_attempts', a['avgsum_attempts'])
print('avgsum_durations', a['avgsum_durations'])
# print(avg_data(read_repair_stats_file(sys.argv[1])))
