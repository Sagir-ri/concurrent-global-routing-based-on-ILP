#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* vs ILP Routing Comparison Tool
对比A*算法和ILP算法的布线结果差异

使用方法:
python routing_compare.py --astar mips_core_astar.ap --ilp mips_core_ilp.ap
或者
python routing_compare.py --dir1 astar_results --dir2 ilp_results
"""

import os
import sys
import argparse
import re
from collections import defaultdict, Counter
import json
from datetime import datetime

class APFileParser:
    """Alliance AP文件解析器"""
    
    def __init__(self, filepath):
        self.filepath = filepath
        self.header = {}
        self.area = None
        self.instances = []
        self.connections = []
        self.segments = []
        self.vias = []
        self.references = []
        
    def parse(self):
        """解析AP文件"""
        try:
            with open(self.filepath, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
        except Exception as e:
            print(f"Error reading {self.filepath}: {e}")
            return False
            
        for line_num, line in enumerate(lines, 1):
            line = line.strip()
            if not line:
                continue
                
            try:
                if line.startswith('V ALLIANCE'):
                    self.header['version'] = line
                elif line.startswith('H '):
                    self.header['header'] = line
                elif line.startswith('A '):
                    self._parse_area(line)
                elif line.startswith('I '):
                    self._parse_instance(line)
                elif line.startswith('C '):
                    self._parse_connection(line)
                elif line.startswith('S '):
                    self._parse_segment(line)
                elif line.startswith('V '):
                    self._parse_via(line)
                elif line.startswith('R '):
                    self._parse_reference(line)
            except Exception as e:
                print(f"Warning: Error parsing line {line_num} in {self.filepath}: {e}")
                print(f"Line content: {line}")
                
        return True
    
    def _parse_area(self, line):
        """解析面积定义: A x1,y1,x2,y2"""
        parts = line.split()
        if len(parts) >= 2:
            coords = parts[1].split(',')
            if len(coords) >= 4:
                self.area = {
                    'x1': int(coords[0]),
                    'y1': int(coords[1]), 
                    'x2': int(coords[2]),
                    'y2': int(coords[3])
                }
    
    def _parse_instance(self, line):
        """解析实例: I x,y,model,name,transformation"""
        parts = line.split(',')
        if len(parts) >= 5:
            self.instances.append({
                'x': int(parts[0].split()[1]),
                'y': int(parts[1]),
                'model': parts[2],
                'name': parts[3],
                'transform': parts[4]
            })
    
    def _parse_connection(self, line):
        """解析连接: C x,y,width,signal,layer_num,direction,layer"""
        parts = line.split(',')
        if len(parts) >= 7:
            self.connections.append({
                'x': int(parts[0].split()[1]),
                'y': int(parts[1]),
                'width': int(parts[2]),
                'signal': parts[3],
                'layer_num': int(parts[4]),
                'direction': parts[5],
                'layer': parts[6]
            })
    
    def _parse_segment(self, line):
        """解析线段: S x1,y1,x2,y2,width,signal,direction,layer"""
        parts = line.split(',')
        if len(parts) >= 8:
            self.segments.append({
                'x1': int(parts[0].split()[1]),
                'y1': int(parts[1]),
                'x2': int(parts[2]),
                'y2': int(parts[3]),
                'width': int(parts[4]),
                'signal': parts[5],
                'direction': parts[6],
                'layer': parts[7]
            })
    
    def _parse_via(self, line):
        """解析通孔: V x,y,type,name"""
        parts = line.split(',')
        if len(parts) >= 4:
            self.vias.append({
                'x': int(parts[0].split()[1]),
                'y': int(parts[1]),
                'type': parts[2],
                'name': parts[3]
            })
    
    def _parse_reference(self, line):
        """解析参考: R x,y,type,name"""
        parts = line.split(',')
        if len(parts) >= 4:
            self.references.append({
                'x': int(parts[0].split()[1]),
                'y': int(parts[1]),
                'type': parts[2],
                'name': parts[3]
            })

class RoutingComparator:
    """布线结果对比器"""
    
    def __init__(self, astar_file, ilp_file):
        self.astar_file = astar_file
        self.ilp_file = ilp_file
        self.astar_data = APFileParser(astar_file)
        self.ilp_data = APFileParser(ilp_file)
        
    def compare(self):
        """执行对比分析"""
        print("="*80)
        print("A* vs ILP Routing Comparison Analysis")
        print("="*80)
        print(f"A* File: {self.astar_file}")
        print(f"ILP File: {self.ilp_file}")
        print()
        
        # 解析文件
        print("Parsing files...")
        if not self.astar_data.parse():
            print("Failed to parse A* file")
            return
        if not self.ilp_data.parse():
            print("Failed to parse ILP file")
            return
        print("Parsing completed.\n")
        
        # 基本统计对比
        self._compare_basic_stats()
        
        # 布线质量对比
        self._compare_routing_quality()
        
        # 布线路径对比
        self._compare_routing_paths()
        
        # 金属层使用对比
        self._compare_layer_usage()
        
        # 通孔使用对比
        self._compare_via_usage()
        
        # 连接性对比
        self._compare_connectivity()
        
        # 生成详细报告
        self._generate_detailed_report()
    
    def _compare_basic_stats(self):
        """基本统计对比"""
        print("📊 BASIC STATISTICS COMPARISON")
        print("-" * 50)
        
        astar_stats = {
            'instances': len(self.astar_data.instances),
            'connections': len(self.astar_data.connections),
            'segments': len(self.astar_data.segments),
            'vias': len(self.astar_data.vias)
        }
        
        ilp_stats = {
            'instances': len(self.ilp_data.instances),
            'connections': len(self.ilp_data.connections),
            'segments': len(self.ilp_data.segments),
            'vias': len(self.ilp_data.vias)
        }
        
        print(f"{'Metric':<15} {'A*':<10} {'ILP':<10} {'Difference':<12} {'Change %'}")
        print("-" * 60)
        
        for metric in astar_stats:
            astar_val = astar_stats[metric]
            ilp_val = ilp_stats[metric]
            diff = ilp_val - astar_val
            change_pct = (diff / astar_val * 100) if astar_val > 0 else 0
            
            print(f"{metric:<15} {astar_val:<10} {ilp_val:<10} {diff:<12} {change_pct:>7.1f}%")
        
        print()
    
    def _compare_routing_quality(self):
        """布线质量对比"""
        print("🎯 ROUTING QUALITY COMPARISON")
        print("-" * 50)
        
        # 计算总线长
        astar_length = sum(abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                          for s in self.astar_data.segments)
        ilp_length = sum(abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                        for s in self.ilp_data.segments)
        
        # 计算通孔数量
        astar_vias = len(self.astar_data.vias)
        ilp_vias = len(self.ilp_data.vias)
        
        print(f"Total Wire Length:")
        print(f"  A*:  {astar_length:,} units")
        print(f"  ILP: {ilp_length:,} units")
        print(f"  Reduction: {astar_length - ilp_length:,} units ({(astar_length - ilp_length)/astar_length*100:.1f}%)")
        print()
        
        print(f"Total Vias:")
        print(f"  A*:  {astar_vias:,}")
        print(f"  ILP: {ilp_vias:,}")
        print(f"  Reduction: {astar_vias - ilp_vias:,} ({(astar_vias - ilp_vias)/astar_vias*100:.1f}%)")
        print()
        
        # 计算拥塞度 (segments per unit area)
        if self.astar_data.area and self.ilp_data.area:
            astar_area = (self.astar_data.area['x2'] - self.astar_data.area['x1']) * \
                        (self.astar_data.area['y2'] - self.astar_data.area['y1'])
            ilp_area = (self.ilp_data.area['x2'] - self.ilp_data.area['x1']) * \
                      (self.ilp_data.area['y2'] - self.ilp_data.area['y1'])
            
            astar_congestion = len(self.astar_data.segments) / astar_area * 1e6
            ilp_congestion = len(self.ilp_data.segments) / ilp_area * 1e6
            
            print(f"Routing Density (segments per mm²):")
            print(f"  A*:  {astar_congestion:.2f}")
            print(f"  ILP: {ilp_congestion:.2f}")
            print()
    
    def _compare_routing_paths(self):
        """布线路径对比"""
        print("🛣️  ROUTING PATH COMPARISON")
        print("-" * 50)
        
        # 分析路径方向分布
        astar_directions = Counter(s['direction'] for s in self.astar_data.segments)
        ilp_directions = Counter(s['direction'] for s in self.ilp_data.segments)
        
        print("Path Direction Distribution:")
        all_directions = set(astar_directions.keys()) | set(ilp_directions.keys())
        
        for direction in sorted(all_directions):
            astar_count = astar_directions.get(direction, 0)
            ilp_count = ilp_directions.get(direction, 0)
            print(f"  {direction:<8}: A* {astar_count:<6} | ILP {ilp_count:<6}")
        print()
        
        # 分析线段长度分布
        astar_lengths = [abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                        for s in self.astar_data.segments]
        ilp_lengths = [abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                      for s in self.ilp_data.segments]
        
        if astar_lengths and ilp_lengths:
            print("Segment Length Statistics:")
            print(f"  A*  - Avg: {sum(astar_lengths)/len(astar_lengths):.1f}, "
                  f"Max: {max(astar_lengths)}, Min: {min(astar_lengths)}")
            print(f"  ILP - Avg: {sum(ilp_lengths)/len(ilp_lengths):.1f}, "
                  f"Max: {max(ilp_lengths)}, Min: {min(ilp_lengths)}")
        print()
    
    def _compare_layer_usage(self):
        """金属层使用对比"""
        print("🏗️  METAL LAYER USAGE COMPARISON")
        print("-" * 50)
        
        astar_layers = Counter(s['layer'] for s in self.astar_data.segments)
        ilp_layers = Counter(s['layer'] for s in self.ilp_data.segments)
        
        all_layers = set(astar_layers.keys()) | set(ilp_layers.keys())
        
        print(f"{'Layer':<10} {'A* Count':<10} {'ILP Count':<10} {'Difference':<12}")
        print("-" * 45)
        
        for layer in sorted(all_layers):
            astar_count = astar_layers.get(layer, 0)
            ilp_count = ilp_layers.get(layer, 0)
            diff = ilp_count - astar_count
            print(f"{layer:<10} {astar_count:<10} {ilp_count:<10} {diff:<12}")
        print()
    
    def _compare_via_usage(self):
        """通孔使用对比"""
        print("🔗 VIA USAGE COMPARISON")
        print("-" * 50)
        
        astar_via_types = Counter(v['type'] for v in self.astar_data.vias)
        ilp_via_types = Counter(v['type'] for v in self.ilp_data.vias)
        
        all_via_types = set(astar_via_types.keys()) | set(ilp_via_types.keys())
        
        if all_via_types:
            print(f"{'Via Type':<15} {'A* Count':<10} {'ILP Count':<10} {'Difference'}")
            print("-" * 50)
            
            for via_type in sorted(all_via_types):
                astar_count = astar_via_types.get(via_type, 0)
                ilp_count = ilp_via_types.get(via_type, 0)
                diff = ilp_count - astar_count
                print(f"{via_type:<15} {astar_count:<10} {ilp_count:<10} {diff}")
        else:
            print("No via information found in files.")
        print()
    
    def _compare_connectivity(self):
        """连接性对比"""
        print("🔌 CONNECTIVITY COMPARISON")
        print("-" * 50)
        
        # 分析信号连接
        astar_signals = set(c['signal'] for c in self.astar_data.connections)
        ilp_signals = set(c['signal'] for c in self.ilp_data.connections)
        
        common_signals = astar_signals & ilp_signals
        astar_only = astar_signals - ilp_signals
        ilp_only = ilp_signals - astar_signals
        
        print(f"Signal Analysis:")
        print(f"  Common signals: {len(common_signals)}")
        print(f"  A* only signals: {len(astar_only)}")
        print(f"  ILP only signals: {len(ilp_only)}")
        
        if astar_only:
            print(f"  A* unique: {list(astar_only)[:5]}{'...' if len(astar_only) > 5 else ''}")
        if ilp_only:
            print(f"  ILP unique: {list(ilp_only)[:5]}{'...' if len(ilp_only) > 5 else ''}")
        print()
    
    def _generate_detailed_report(self):
        """生成详细报告"""
        print("📋 DETAILED ANALYSIS SUMMARY")
        print("-" * 50)
        
        # 判断是否相同
        files_identical = self._files_are_identical()
        
        if files_identical:
            print("🟢 RESULT: Files are IDENTICAL")
            print("   The A* and ILP algorithms produced the same routing result.")
        else:
            print("🔴 RESULT: Files are DIFFERENT")
            print("   The ILP algorithm produced a different routing result than A*.")
            
            # 性能评估
            self._evaluate_performance()
        
        # 保存报告到文件
        self._save_report_to_file()
    
    def _files_are_identical(self):
        """检查文件是否完全相同"""
        try:
            with open(self.astar_file, 'rb') as f1, open(self.ilp_file, 'rb') as f2:
                return f1.read() == f2.read()
        except:
            return False
    
    def _evaluate_performance(self):
        """评估性能改进"""
        print("\n🏆 PERFORMANCE EVALUATION:")
        
        # 线长比较
        astar_length = sum(abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                          for s in self.astar_data.segments)
        ilp_length = sum(abs(s['x2'] - s['x1']) + abs(s['y2'] - s['y1']) 
                        for s in self.ilp_data.segments)
        
        if astar_length > 0:
            length_improvement = (astar_length - ilp_length) / astar_length * 100
            if length_improvement > 0:
                print(f"   ✅ Wire length reduced by {length_improvement:.1f}%")
            else:
                print(f"   ❌ Wire length increased by {-length_improvement:.1f}%")
        
        # 通孔比较
        astar_vias = len(self.astar_data.vias)
        ilp_vias = len(self.ilp_data.vias)
        
        if astar_vias > 0:
            via_improvement = (astar_vias - ilp_vias) / astar_vias * 100
            if via_improvement > 0:
                print(f"   ✅ Via count reduced by {via_improvement:.1f}%")
            else:
                print(f"   ❌ Via count increased by {-via_improvement:.1f}%")
    
    def _save_report_to_file(self):
        """保存报告到JSON文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = f"routing_comparison_{timestamp}.json"
        
        report_data = {
            'timestamp': timestamp,
            'astar_file': self.astar_file,
            'ilp_file': self.ilp_file,
            'astar_stats': {
                'instances': len(self.astar_data.instances),
                'connections': len(self.astar_data.connections),
                'segments': len(self.astar_data.segments),
                'vias': len(self.astar_data.vias)
            },
            'ilp_stats': {
                'instances': len(self.ilp_data.instances),
                'connections': len(self.ilp_data.connections),
                'segments': len(self.ilp_data.segments),
                'vias': len(self.ilp_data.vias)
            },
            'identical': self._files_are_identical()
        }
        
        try:
            with open(report_file, 'w') as f:
                json.dump(report_data, f, indent=2)
            print(f"\n📄 Detailed report saved to: {report_file}")
        except Exception as e:
            print(f"\n❌ Failed to save report: {e}")

def compare_directories(dir1, dir2):
    """对比两个目录中的所有AP文件"""
    print("📁 DIRECTORY COMPARISON MODE")
    print("="*80)
    
    # 查找AP文件
    ap_files1 = {f for f in os.listdir(dir1) if f.endswith('.ap')}
    ap_files2 = {f for f in os.listdir(dir2) if f.endswith('.ap')}
    
    common_files = ap_files1 & ap_files2
    only_in_dir1 = ap_files1 - ap_files2
    only_in_dir2 = ap_files2 - ap_files1
    
    print(f"Directory 1: {dir1}")
    print(f"Directory 2: {dir2}")
    print(f"Common files: {len(common_files)}")
    print(f"Only in dir1: {len(only_in_dir1)}")
    print(f"Only in dir2: {len(only_in_dir2)}")
    print()
    
    if only_in_dir1:
        print(f"Files only in {dir1}: {list(only_in_dir1)}")
    if only_in_dir2:
        print(f"Files only in {dir2}: {list(only_in_dir2)}")
    print()
    
    # 对比共同文件
    for filename in sorted(common_files):
        file1 = os.path.join(dir1, filename)
        file2 = os.path.join(dir2, filename)
        
        print(f"\n🔍 Comparing {filename}:")
        print("-" * 60)
        
        comparator = RoutingComparator(file1, file2)
        comparator.compare()

def main():
    parser = argparse.ArgumentParser(description='Compare A* and ILP routing results')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--files', nargs=2, metavar=('ASTAR_FILE', 'ILP_FILE'),
                      help='Compare two specific AP files')
    group.add_argument('--dirs', nargs=2, metavar=('ASTAR_DIR', 'ILP_DIR'),
                      help='Compare all AP files in two directories')
    
    args = parser.parse_args()
    
    if args.files:
        astar_file, ilp_file = args.files
        if not os.path.exists(astar_file):
            print(f"Error: A* file not found: {astar_file}")
            return
        if not os.path.exists(ilp_file):
            print(f"Error: ILP file not found: {ilp_file}")
            return
            
        comparator = RoutingComparator(astar_file, ilp_file)
        comparator.compare()
        
    elif args.dirs:
        dir1, dir2 = args.dirs
        if not os.path.isdir(dir1):
            print(f"Error: Directory not found: {dir1}")
            return
        if not os.path.isdir(dir2):
            print(f"Error: Directory not found: {dir2}")
            return
            
        compare_directories(dir1, dir2)

if __name__ == '__main__':
    main()