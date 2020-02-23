#include<stdio.h>
#include<conio.h>
#define INFINITY 9999

int arena(int start,int end)
{
    int i,j;

    int arr[7][3]=
    {

        {
            13,14,15
        },
        {
            12,-1,11
        },
        {
            9,-1,10
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

    if(arr[i][j]==start)
    {

        if(arr[i-1][j]==end && i-1!=-1)
            return 3;
        else if(arr[i+1][j]==end && i+1 !=7)
            return 4;
        else if(arr[i][j-1]==end && j-1 !=-1)
            return 1;
        else
            return 2;
    }
}

}
void dijkstra(int G[16][16],int n,int startnode,int endnode);

int main()
{

	int i,j,n,u,e;
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



	printf("\nEnter the starting node:");
	scanf("%d",&u);
	printf("\nEnter the ending node:");
	scanf("%d",&e);
	dijkstra(G,16,u,e);

	return 0;
}

void dijkstra(int G[16][16],int n,int startnode,int endnode)
{
    int val,co=0;
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

	distance[startnode]=0;
	visited[startnode]=1;
	count=1;

	while(count<n-1)
	{
		mindistance=INFINITY;


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
			printf("\nDistance of node%d=%d\n\n",i,distance[i]);
			arr[co]=i;
			co++;

			j=i;
			do
			{
				j=pred[j];
                arr[co]=j;
                co++;
             ;
			}while(j!=startnode);
	    }
        for(int y=co;y<16;y++)
            arr[y]=-1;
	for(int i=15;i>0;i--)
    {
        if(arr[i]!=-1)
        {
        val = arena(arr[i],arr[i-1]);
        if(val==1)
        printf("Left\t");
        else if(val==2)
        printf("Right\t");
        else if(val==3)
        printf("Up\t");
        else
        printf("Down\t");
        }
    }
}
