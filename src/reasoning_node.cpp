#include <ros/ros.h>
#include <iostream>
#include <string>

#include <rosprolog/rosprolog_client/PrologClient.h>

#include <world_percept_assig3/UpdateObjectList.h>

using namespace std;

class Reasoner
{
private: 
    PrologClient pl_;
    int ID_;

    std::string srv_assert_knowledge_name_;
    ros::ServiceServer assert_knowledge_srv_;                            // Advertise service to assert knowledge in the ontology
public:

    Reasoner(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("Wait for the Prolog service...");

        if(pl_.waitForServer())
            pl_ = PrologClient("/rosprolog", true);

        ID_=0; //Global variable to include in the asserted instances

        srv_assert_knowledge_name_ = "assert_knowledge";
        assert_knowledge_srv_ = nh.advertiseService(srv_assert_knowledge_name_, &Reasoner::srv_assert_callback, this);
    };

    ~Reasoner(){

    };

private:    


     /**
     * @brief Callback function for the service that adds objects to the map_objects list
     *
     * @param Request requested object to be added to the knowledge base
     * @param Respose response from the service when the object has been asserted (true/false)
     */
    bool srv_assert_callback(world_percept_assig3::UpdateObjectList::Request &req,
                             world_percept_assig3::UpdateObjectList::Response &res)
    {
        ROS_INFO_STREAM("Got new object: " << req.object_name);
        std::string object;
        
        object=req.object_name;
        res.confirmation = false;

        getClass(object);

        if(assertKnowledge(object))
        {
            res.confirmation = true;
            ++ID_;
        }


        return res.confirmation;
    }


    void getClass(std::string className)
    {
        
        std::string query = "get_class('" + className + "').";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        for (PrologQuery::iterator it = bdgs.begin(); it != bdgs.end(); ++it) 
        {
            break;
        }

        bdgs.finish();
       
    }

    bool assertKnowledge(std::string className)
    {
        std::string instanceName;

        std::string query = "create_instance_from_class('" + className + "', " + std::to_string(ID_) + ", Instance).";

        ROS_INFO_STREAM("query: "<<query);

        PrologQuery bdgs = pl_.query(query);

        bool result = false;
        for(PrologQuery::iterator it=bdgs.begin(); it != bdgs.end(); ++it)
        {
            result = true;
            break;
        }

        bdgs.finish();

        if(result)
        {
            instanceName = "http://www.chalmers.se/ontologies/ssy236Ontology.owl#" + className + "_" + std::to_string(ID_);
            ROS_WARN_STREAM("new instance in knowledge base: "<<instanceName);
        }
        
        return result;
    }

}; //class Reasoner

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "reasoning_node");

  ros::NodeHandle nh;   
  
  Reasoner myReasoner(nh);

  ros::spin();

  
  return 0;
}
