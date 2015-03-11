FILE=xyzbruteforceconfig.txt
for i in $(seq 0 1 20); do
	sed -i "/neighbors/c\\neighbors $i"  $FILE
	./bin/pct_cnn_demo --t ../../faces/face4.pcd --s ../../faces/face2.pcd --f $FILE 
	mv screen.png "screen-$i.png"
done


