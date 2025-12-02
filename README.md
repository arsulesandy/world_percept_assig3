

Group Members: Group 29
- Student 1: Sandeep Arsule
- Student 2: Harshith Pidur Kuppusamy


Build the Workspace

Inside the container:

source /knowrob_ws/devel/setup.bash
catkin_make
source devel/setup.bash



Run the Simulation

source /tiago_public_ws/devel/setup.bash
roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true world:=simple_office_with_people gui:=false



Run the Reasoning System

docker exec -it -w /home/user/exchange/ssy236_arsule tiago bash
source /knowrob_ws/devel/setup.bash
source devel/setup.bash
roslaunch world_percept_assig3 reasoning.launch



Verify Task 1 – Ontology Classes

rosrun rosprolog rosprolog world_percept_assig3

?- owl_subclass_of(ssy236Ontology:'Table', _).
true .

?- owl_subclass_of(ssy236Ontology:'Table', _).
true .

?- owl_subclass_of(ssy236Ontology:'Table', _).
true .

?- owl_subclass_of(ssy236Ontology:'Trash_bin', _).
true .

?- owl_subclass_of(ssy236Ontology:'Floor', _).
true .

?- owl_subclass_of(ssy236Ontology:'Cereal', _).
true


Removed classes (explicit triple check):


?- rdf(ssy236Ontology:'Bookshelf', rdf:type, owl:'Class').
false.

?- rdf(ssy236Ontology:'Bowl', rdf:type, owl:'Class').
false.

?- rdf(ssy236Ontology:'Coke_can', rdf:type, owl:'Class').
false.

?- rdf(ssy236Ontology:'TestClass', rdf:type, owl:'Class').
false.




Verify Task 2 – Prolog get_class/1

?- get_class('TestClass').
New class created: TestClass
true.

?- get_class('TestClass').
false.

?- rdf(ssy236Ontology:'TestClass', rdf:type, owl:'Class').
true.


in the above step 2 false, means class already exists in ontology and true means we created new class i.e. Statement - New class created: TestClass

Verify Task 3 – Reasoning Node + Service

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosservice list | grep assert
/assert_knowledge

Call the service:

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosservice call /assert_knowledge "object_pose:
>   position: {x: 1.0, y: 1.0, z: 0.5}
>   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
> object_name: 'Beer'"

confirmation: True



Group of Two Students:
- Sandeep Arsule
- Harshith Pidur Kuppusamy


