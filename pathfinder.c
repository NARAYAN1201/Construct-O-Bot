
//header files for adding c functionality
#include<stdio.h>
#include<conio.h>
#define INFINITY 9999

//declarations of the functions

void dijkstra(int G[16][16],int n,int startnode,int endnode);
void h(int G[16][16],int house);
int arena(int start, int end);
void eyantra();

//defining global variables 

int floor_array[5]={1,0,0,1,0};//for taking the priority for high rise house (1) and low level(0)
int house_total_requirement[5] = {2,2,2,1,2}; //represent no . of requirement of boxes
int which_material[10]={0,0,0,0,0,0,0,0,2,1};//represent the address of blocks 
int left,right,up,down;//directions variables defined globally for everywhere accessible
int current_node = 1; //for checking up the current status of bot based on position

//function for setting the directions
void set_direction()
{
    left =0; //taking left ,right ,up and down directions
    right = 0;
    up =0;
    down =1;//by default we are taking downward direction
}

//tasking functions
void leftside()
{
	printf("TURN 90 left\n");
}


void rightside()
{
    printf("Turn 90 Right\n");
}


void pick()
{
	printf("Picking\n");
}


void place()
{
	printf("placing\n");
}

void forward()
{
	printf("moving forward\n");
}

//path regarding/tasking functions help to decide house . it will decide ,where we have to place the box  
void house_decision(int G[16][16],int house)//in house variable we are sending the no. of house
{
         if(house ==1) //condition for house one
           {
                dijkstra(G,16,current_node,5);
                current_node=5;//changing the currrent status
                if(up==1)
                  {
                    leftside();
                    left =1;
                    up =0;    //for changing the previous diection ,adding 0  to previous direction and  1 to new direction
                  }
                if(down == 1)
                  {
                     rightside();//rotating along rightside
                     down =0;
                     left=1;
                  }

            }
          else if(house ==2)//condition for house 2
            {
                    dijkstra(G,16,current_node,6);
                    current_node=6;
                    if(up==1)
                     {
                          up=0;
                          right=1;//for changing the direction
                          rightside();
                     }
                     if(down ==1)
                       {
                          down=0;
                          right =1;
                          leftside();
                       }
             }
            else if(house ==3)//condition for house 3
             {
                    dijkstra(G,16,current_node,9);
                    current_node=9;
                    if(up==1)
                    {
                      leftside();//rotate along left side
                      left =1;
                      up =0;
                    }
                    if(down == 1)
                    {
                       rightside();
                       down =0;
                       left=1;
                    }
              }
            else if(house ==4)//condition for house 4
              {
                    dijkstra(G,16,current_node,10);
                    current_node=10;//changing the current node position 
                    if(up==1)
                      {
                          up=0;
                          right=1;
                          rightside();//rightside function to rotate along right side

                      }
                    if(down ==1)
                      {
                          down=0;
                          right =1;
                          leftside();//leftside fuction to rotate along left side
                      }
                }
            else//condition for house 5 (taking default becauuse no any house is left more)
                 {
                     dijkstra(G,16,current_node,14);//calling this function we are getting correct path
                     current_node=14;
                     if(left ==1)
                     {
                         left =0;
                         leftside();
                         down =1;
                     }
                     if(right ==1)
                     {
                         right = 0;
                         rightside();
                         down=1;//changing the direction 
                     }
                  }
                //upper all the conditions just send us to specific position where we have to perform placing
                 place();//place function to perform placing 

}


//defining the function arena to get direction of end from the start
int arena(int start,int end) //
{
    int i,j;
    //specifiy the special cases in arena based on specials conections 1.wall connection and 2. Zig-Zag connection
    if(start ==5 && end == 6)
        return  2;
    if(start == 6 && end ==5)
        return 1;
    if(start == 9 && end == 10)
        return 2;
    if(start == 10 && end ==9)
        return 1;
    int arr[7][3]=
    {

        {
            13,14,15 //14 is an intermidatery node between 13 and 15
        },
        {
            12,-1,11
        },
        {
            9,-1,10 //if there is no any intermidate node betwe 2 nodes then we add it as -1
        },
        {
            8,-1,7
        },
        {
            5,-1,6
        },
        {

            4,-1,3
        },
        {

            0,1,2
        }
    };
for(i =0;i<7;i++)
  for( j=0;j<3;j++)
  {

    if(arr[i][j]==start) //first we find node with whom we have to make connection
    {

           if(arr[i-1][j]==end && i-1!=-1)
             return 3;//3 for up direction
           else if(arr[i+1][j]==end && i+1 !=7)
             return 4; //4 for downward direction 
           else if(arr[i][j-1]==end && j-1 !=-1)
             return 1;//1 for left
           else
             return 2;//2 for right
    }
  }

}

//Dijkstra algorithm for giving the correct path between the given two nodes startnode and endnode
void dijkstra(int G[16][16],int n,int startnode,int endnode)
{
    int val,co=0,y;   //this is whole dijkstra algorithm for finding the minimum distance between the given nodes
    int arr[16]={214};
    int cost[16][16],distance[16],pred[16];
	int visited[16],count,mindistance,nextnode,i,j;
    for(i=0;i<n;i++)
    
		for(j=0;j<n;j++)   
			if(G[i][j]==0)
				cost[i][j]=INFINITY;
			else
				cost[i][j]=G[i][j];
    for(i=0;i<n;i++)
	 {
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	 }
    distance[startnode]=0;//used to find distance
	visited[startnode]=1;//for defining wheathe we have visited to the node otr not
	count=1;
    while(count<n-1)
	{
		mindistance=INFINITY;//defining maximum distacne first then we will set min distance 


		for(i=0;i<n;i++)
			if(distance[i]<mindistance&&!visited[i])
			{
				mindistance=distance[i];
				nextnode=i;
			}


			visited[nextnode]=1;
			for(i=0;i<n;i++)
				if(!visited[i])
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
		count++;
	 }

        i= endnode;
		if(i!=startnode)
		{

			arr[co]=i;
			co++;
			j=i;
			do
			{
				j=pred[j];
                arr[co]=j;
                co++;

			}
			while(j!=startnode);
	    }
        for( y=co;y<16;y++)
            arr[y]=-1;
	    for( i=15;i>0;i--)
        {
        if(arr[i]!=-1)
        {
        val = arena(arr[i],arr[i-1]);
    //condition 1
    if(val==1)
        {

          if(right ==1)
          {
        	left= 1;
        	right =0;
        	leftside();
        	leftside();
            forward();
          }
          else if(up ==1)
          {
        	left =1;
        	up =0;
        	leftside();
            forward();
          }
		  else if (down==1)
		  {
			left =1;
			down =0;
			rightside();
            forward();
          }
		  else
		  forward();

       }

      //condition 2
    else if(val==2)
    {

            if(right ==1)
          	forward();
            else if(up ==1)
            {
              up=0;
        	  right =1;
        	  rightside();
              forward();
		    }
		    else if (down==1)
		    {
			  down=0;
			  right = 1;
			  leftside();
              forward();
		    }
            else
		    {
		        left=0;
		     	right =1;
		      	rightside();
			    rightside();
                forward();
            }
        }

   //condition 3
    else if(val==3)
    {

            if(left ==1)
            {
              left =0;
        	  up =1;
        	  rightside();
              forward();
            }
		    else if (right ==1)
		    {
		        right =0;
		     	up =1;
		      	leftside();
                forward();
            }
		    else if (down ==1)
            {
                down =0;
		     	up =1;
			    leftside();
			    leftside();
                forward();
		    }
		    else
		     	forward();
    }

    //condition 4
    else
    {

        if(up==1)
          {
            up=0;
        	down =1;
        	leftside();
        	leftside();
            forward();
		  }
		else  if (left ==1)
		  {
		    left =0;
			down =1;
			leftside();
            forward();
		  }

		else if (right ==1)
		  {
		    right =0;
			down =1;
			rightside();
            forward();
		  }
		else
		  	forward();
    }
         }
      }
}

void eyantra()//function used to define conection between the nodes if connection is present then it will 1 otherwise it will 0
{
    int i,house,rem;
    int G[16][16]={
	    {

	        0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0//
	    },
	    {

	        1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0
	    },
	    {

	        1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,1,0,2,0,1,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,1,0,2,0,1,0,0,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,0,1,0,3,0,1,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,1,0,3,0,1,0,0,0,0
	    },
	    {

	        0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1
	    },{

	        0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1
	    },{

	        0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0
	    }
	};

    for(i=9;i>=0;i--)
    {
        //first zero check condition
        if(which_material[i]==0)
            continue;
        // first condition
        if(which_material[i]==1 || which_material[i]==2)
        {
            dijkstra(G,16,current_node,4);
            current_node = 4;
            if(which_material[i]==1)
            {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==2)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                 //placing condition
                house_decision(G,house);

             }
        }

        //second condition

        if(which_material[i]==3 || which_material[i]==4)
        {
            dijkstra(G,16,current_node,3);
            current_node = 3;

             if(which_material[i]==3)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;
               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==4)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);

              }
         }

    //third condition
        if(which_material[i]==5 || which_material[i]==6)
        {
            dijkstra(G,16,current_node,8);
            current_node = 8;
             if(which_material[i]==5)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house = (i/2)+1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==6)
             {
               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;
               }
               pick();
               house = (i/2) +1;
               //placing condition
               house_decision(G,house);
            }
        }

        //fourth condition

        if(which_material[i]==7 || which_material[i]==8)
        {
            dijkstra(G,16,current_node,7);
            current_node = 7;
            if(which_material[i]==7)
            {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house =(i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==8)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
            }
         }
        //fifth condition
       if(which_material[i]==9 || which_material[i]==10)
          {
             dijkstra(G,16,current_node,12);
             current_node = 12;
             if(which_material[i]==9)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;
               }
                 pick();
                 house = (i/2) +1;
                 //placing condition
                 house_decision(G,house);
             }
               if(which_material[i]==10)
               {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);
                }
        }

        //sixth condition
        if(which_material[i]==11 || which_material[i]==12)
        {
             dijkstra(G,16,current_node,11);
            current_node = 11;
             if(which_material[i]==11)
             {
               if(up==1)
               {
                    leftside();
                    left =1;
                    up= 0;
               }
               if(down == 1)
               {
                 rightside();
                 left =1;
                 down =0;

               }
                 pick();
                 house =(i/2)+1;
                 //placing condition
                 house_decision(G,house);
             }
             if(which_material[i]==12)
             {

               if(down==1)
               {
                    leftside();
                    right =1;
                    down= 0;
               }
               if(up == 1)
               {
                 rightside();
                 right =1;
                 up =0;

               }
                pick();
                house = (i/2) +1;
                //placing condition
                house_decision(G,house);

             }
        }
    }
}

//driver code is here
int main()
{
    set_direction();//function to define predefined directions 
    eyantra();//function for defining whole condtion to traverse all the arena
    return 0;
}



