# lis_vision
Very experimental &amp; less documented code that's not guaranteed to work!


### Simple vision code
`roslaunch lis_vision simple_vision.launch`

### open rviz and subscribe to the following:

MarkerArray
* /bounding_boxes
* /filtered_clusters
* /tabletop/clusters

ORKTable
* /table_array
* /filtered_table_array

----

launching simple_vision will start several nodes

*  object_recognition_ros server 

    * uses launch/detection.ork as a configuration file
    * this server won't do anything unless it is called

* lis_pr2_pkg client

    * built off of object_recognition_ros client 
    * continuously call the server to report object dections


The client and server nodes will report a lot of table detections and
object clusters.  You can see them in rviz under /table_array and 
/tabletop/clusters 
The next nodes that are launched do simple processing of these messages to
only report the ones that are likely to be a table and object we are 
interested in

* lis_pr2_pkg filter_tables.py
    
    * listes to /table_array and determines which one is best table
    * publishes to /filtered_table_array

* lis_pr2_pkg detect_clusters.py
    
    * listens to the best table from filter tables to determine if 
    clusters in /tabletop/clusters are relevant
    * republishes clusters that meets simple criteria (/flitered_clusters)
    * draws axis-aligned bounding boxes around 
    filtered clusters (/bounding_boxes)




