import os
import re
from statistics import mean, stdev

def parse_runtime(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
        if 'Lead-off Latency:' not in content:
            return None
        # Find the line with Total runtime
        match = re.search(r'Total runtime:\s*(\d+\.?\d*)\s*s', content)
        if match:
            return float(match.group(1))
    return None

def main():
    output_dir = 'output'  # Relative path from utils
    runtimes = []
    for filename in os.listdir(output_dir):
        if filename.endswith('.out'):
            filepath = os.path.join(output_dir, filename)
            runtime = parse_runtime(filepath)
            if runtime is not None:
                runtimes.append(runtime)
    
    if runtimes:
        avg = mean(runtimes)
        std = stdev(runtimes) if len(runtimes) > 1 else 0
        print(f'Average total runtime: {avg:.1f} ± {std:.1f} s')
    else:
        print('No valid .out files found with Lead-off Latency.')

if __name__ == '__main__':
    main()
