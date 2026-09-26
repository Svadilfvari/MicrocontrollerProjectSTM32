#include <stdio.h>
#include<stdlib.h>

#define DIMX 20
#define DIMY 20

void init_cart( int Nb_tab,int** tableau);
void deplace_perso( int Nb_perso,int Nb_tab,int** tableau, int* tab_c);
void affiche_carte (int Nb_perso,int Nb_tab,int** tableau, int* tab_c);

int main ()
{
    int* tab_c=malloc ( 2 * sizeof ( float* ) );;

    int ** tableau ;
    tableau = malloc ( DIMX * sizeof ( float* ) );
    int i;
    for ( i = 0; i < DIMX; i++ )
    {
       tableau[i] = calloc ( DIMY, sizeof ( float ) );
    }

    int x,y;
    tab_c[0]=0;
    tab_c[1]=0;

    init_cart( 20,tableau);
    affiche_carte (2,20,tableau, tab_c);

    deplace_perso( 2,20,tableau,tab_c);




    return 0;
}

void init_cart( int Nb_tab,int** tableau)
{
    int i,j;
    for (i = 0 ; i <20; i++) 		 /*giving values*/
    {
        for (j = 0 ; j < 20; j++)
        {
            tableau [i][j] = 0 ;
        }
    }
    tableau [2][3]=5;
    for (i = 0 ; i <20; i++) 		 /*giving values*/
    {
       tableau [i][0]=5;
    }
    tableau [6][5]=3;
    tableau [4][5]=2;
    tableau [7][17]=2;
    tableau [12][7]=2;		/*0=grass, 1=flower, 2=tree,3=rock,4= key,5=gold coin,6=padlock, 7=trap,8=monster*/
    tableau [9][8]=6;
    tableau [15][4]=8;
    tableau [2][18]=8;
    tableau [19][13]=8;
    tableau [15][6]=2;
    tableau [6][11]=3;
    tableau [9][9]=1;
    tableau [19][4]=5;
    tableau [2][3]=5;
    tableau [17][3]=5;
    tableau [2][9]=5;

}

void deplace_perso( int Nb_perso,int Nb_tab, int** tableau, int* tab_c)
{

    int Choice;
    int HP;
    int Gold_coin;
    int x,y;
    int key;

    key = 0;
    HP = 10;
    Gold_coin=0;

    x=0;
    y=0;
    tab_c[0]=x




    ;
    tab_c[1]=y;
    printf("Welcome to the greed island to move to the right direction tap 6, left 4, up 8 , down 2 , to exit 0\n ");




    while ((HP>0)&&(Gold_coin<10))
    {
        scanf("%d",&Choice );

        switch ( Choice)
        {

        case 4 :
            printf(" you are going to the left\n");
            y--;
            if (y < 0)
            {
                printf("You shall not pass\n ");
                y++;
            }
            else
            {
                if  ((tableau [x][y]==8)||(tableau[x][y]==7))
                {
                    HP--;
                    printf("You were beyond a threat thus yous lost 1 HP, your current HP is %d\n",HP);
                    tableau [x][y]==0;
                }

                if ((tableau [x][y]==2)|| (tableau [x][y]==3))
                {
                    printf ("You are beyond an obstacle take another path\n");
                    y++;
                }
                if  (tableau [x][y]==5)
                {
                    Gold_coin++;
                    printf("You found a gold coin keep it up you have currently %d gold coin (s)\n",Gold_coin);
                    tableau [x][y]==0;
                }

                if (tableau [x][y]==6)
                {
                    if (key>=1)
                    {
                        printf("you have some keys so you will open this padlock \n");
                        key--;
                        tableau [x][y]==0;
                    }
                    else
                    {
                        printf("you cannot pass beyond you don't have any keys \n");
                        y++;
                    }

                }

            tab_c[0]=x;
            tab_c[1]=y;
            affiche_carte (   Nb_perso, Nb_tab,tableau,  tab_c );
            }

            break;
        case 8 :
            printf(" you are going to the up");

            x--;
            if ( x<0)
            {
                printf("You shall not pass \n");
                x++;
            }
            else
            {

                if  ((tableau [x][y]==8)||(tableau [x][y]==7))
                {
                    HP--;
                    printf("You were beyond a threat thus you lost 1 HP, your current HP is %d \n",HP);
                    tableau [x][y]==0;
                }

                if ((tableau [x][y]==2)|| (tableau [x][y]==3))
                {
                    printf ("You are beyond an obstacle take another path\n");
                    x++;
                }
                if  (tableau [x][y]==5)
                {
                    Gold_coin++;
                    printf("You found a gold coin keep it up you have currently %d gold coin (s)\n",Gold_coin);
                    tableau [x][y]==0;
                }

                if (tableau [x][y]==6)
                {
                    if (key>=1)
                    {
                        printf("you have some keys so you will open this padlock \n ");
                        key--;
                        tableau [x][y]==0;

                    }
                    else
                    {
                        printf("you cannot pass beyond you don't have any keys \n");
                        x++;

                    }

                }

                tab_c[0]=x;
                tab_c[1]=y;

                affiche_carte (   Nb_perso, Nb_tab,tableau,  tab_c );
            }
            break;
        case 6:
            printf(" you are going to the right \n ");

            y++;
            if ( y >19)
            {
                 y--;
                printf("You shall not pass \n");
            }
            else
            {
                if  ((tableau [x][y]==8)||(tableau [x][y]==7))
                {
                    HP--;
                    printf("You were beyond a threat thus you lost 1 HP, your current HP is %d \n",HP);
                    tableau [x][y]==0;

                }

                if ((tableau [x][y]==2)|| (tableau [x][y]==3))
                {
                    printf ("You are beyond an obstacle take another path \n");
                    y--;
                    printf("\n your coordinate(%d;%d)\n",x,y);

                }
                if  (tableau [x][y]==5)
                {
                    Gold_coin++;
                    printf("You found a gold coin keep it up you have currently %d gold coin (s) \n",Gold_coin);
                    tableau [x][y]==0;

                }

                if (tableau [x][y]==6)
                {
                    if (key>=1)
                    {
                        printf("you have some keys so you will open this padlock \n ");
                        key--;
                        tableau [x][y]=0;

                    }
                    else
                    {
                        printf("you cannot pass beyond you don't have any keys \n ");
                        y--;
                    }

                }

            tab_c[0]=x;
            tab_c[1]=y;

            affiche_carte (   Nb_perso, Nb_tab,tableau,  tab_c );
            }

            break;

        case 2:
            printf(" you are going down \n ");

            x++;
            if ( x >19)
            {
                printf("You shall not pass \n");
                x--;
            }
            else
            {

                if  ((tableau [x][y]==8)||(tableau [x][y]==7))
                {
                    HP--;
                    printf("You were beyond a threat thus you lost 1 HP, your current HP is %d \n",HP);
                    tableau [x][y]=0;
                }
                if ((tableau [x][y]==2)|| (tableau [x][y]==3))
                {
                    printf ("You are beyond an obstacle take another path \n");
                    x--;
                }
                if  (tableau [x][y]==5)
                {
                    Gold_coin++;
                    printf("You found a gold coin keep it up you have currently %d gold coin (s) \n",Gold_coin);
                    tableau [x][y]=0;

                }

                if (tableau [x][y]==6)
                {
                    if (key>=1)
                    {
                        printf("you have some keys so you will open this padlock \n ");
                        key--;
                        tableau [x][y]=0;

                    }
                    else
                    {
                        printf("you cannot pass beyond you don't have any keys \n ");
                    }

                }

            tab_c[0]=x;
            tab_c[1]=y;

            affiche_carte ( Nb_perso, Nb_tab,tableau, tab_c );
            }


            break;
        case 0:
            printf ("you will exit the game ");
        default :
            printf("Wrong number \n");
            break;
        }
    }
    if (HP==0){
        printf("Your greed was not enough \n ");
    }
    else if (Gold_coin==10){
        printf("You are one of the seven deadly sins \n ");
    }

}
void affiche_carte (int Nb_perso,int Nb_tab,int** tableau, int* tab_c)
{
    int x=tab_c[0],y=tab_c[1],i,j;

    printf("\n your coordinate(%d;%d)\n",x,y);


    for (i = 0 ; i < 20 ; i ++)
    {
        for (j = 0 ; j < 20 ; j ++)
        {
            if (x==i&y==j)
            {
                printf("X ");
            }
            else
            {
                printf ("%d ", tableau [i][j]) ;            /* show the map */
            }
        }
         printf ("\n") ;

    }

    printf ("\n") ;

}






















