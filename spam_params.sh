echo " "
echo " "
echo " "
echo " "
cat param_list.txt | while read line
do
   
   VALUE=$(rosparam get $line)
   echo "$line : $VALUE"
done
