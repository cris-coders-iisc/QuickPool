#!/bin/bash
# set -x

rider_count=$1
driver_count=$2
thread_count=$3
run_opt=$4

if test $run_opt = 0
then
	echo "Running QuickPool end-point based matching"
	echo "***********************************************"
	pkill -f quickpool_ED
	run_app=./benchmarks/quickpool_ED
	dir=~/benchmark_data/quickpool_ED
else
	echo "Running QuickPool intersection based matching"
	echo "***********************************************"
	pkill -f quickpool_intersection
	run_app=./benchmarks/quickpool_intersection
	dir=~/benchmark_data/quickpool_intersection
fi
# echo $run_app

# rm -rf $dir/*.log $dir/*.json
mkdir -p $dir

$run_app -p 0 --localhost -r $rider_count -d $driver_count -t $thread_count 2>&1 | tee -a $dir/r_$1_d_$2_0.log &
codes[0]=$!

for rider in $(seq 1 $rider_count)
do
	# echo $rider
	log=$dir/r_$1_d_$2_$rider.log
	if test $rider = 1
	then
		$run_app -p $rider --localhost -r $rider_count -d $driver_count 2>&1 >> $log &
	else
		$run_app -p $rider --localhost -r $rider_count -d $driver_count 2>&1 > /dev/null &
	fi
	codes[$rider]=$!
done

start_driver=$(($rider_count+1))
end_driver=$(($rider_count+$driver_count)) 
for driver in $(seq $start_driver $end_driver)
do
	# echo $driver
	log=$dir/r_$1_d_$2_$driver.log
	if test $driver = $start_driver
	then
		$run_app -p $driver --localhost -r $rider_count -d $driver_count 2>&1 >> $log &
	else
		$run_app -p $driver --localhost -r $rider_count -d $driver_count 2>&1 > /dev/null &
	fi
	codes[$driver]=$!
done

for party in $(seq 0 $end_driver)
do
	wait ${codes[$party]} || return 1
done

