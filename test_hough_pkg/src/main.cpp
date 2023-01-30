

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <cstdlib>
#include <math.h>

#define PI 3.14159265


using namespace cv;
using namespace std;

Mat filter(Mat src,int usedfilter){
    Mat zdst;
    switch (usedfilter){
    case 1:
    // Median Filter:
     cv::medianBlur(src, zdst, 3);
        break;
    case 2:
    // Normalized Box Filter:
     cv::blur(src, zdst, cv::Size(3,3));
        break;
    case 3:
    // Bilateral Filter:
     cv::bilateralFilter(src, zdst, 5, 200.0, 200.0);
        break;
    case 4:
    // Box Filter:
     cv::boxFilter(src, zdst, -1, cv::Size(3.5,3.5));
        break;
    case 5:
    // Gaussian Filter:
     cv::GaussianBlur(src, zdst, cv::Size(5,5),2.0,2);
        break;
    }
    return zdst;
}

double Metrik(double winkel, int length, int vertical, int horizontal, Size image_size){
    double verticalsize = (vertical/((image_size.height * 2) / 3));
    double verticalfactor = (1-abs(1-verticalsize));
    double horizontalfactor = (1-abs(1-(horizontal/(image_size.width / 2))));
    double winkelfactor = (1-abs(1-(winkel/75.0)));
    double metrikcounter = (winkelfactor * length) + (horizontalfactor * horizontal)+ (verticalfactor * vertical);
return metrikcounter;
}

double line_evaluation(const vector<Vec4i> linesP, Size image_size){

    int ankath,gegekath;
    double winkelfilter = 22.5;
    double winkel;
    int length;
    int vertical;
    int horizontal;
    double metrikcounter;
    //cout << "Test:" << endl;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        //cout << "i:" << i << endl;
        // line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        // Liniensortierung: (Horizontal wird aussortiert)
        // Metrikcounter: (anglefactor * length + horizontalfactor * horizontal + verticalfactor * vetical)
        if(l[2]==l[0]){   
            // Senkrechte Linie
            if((image_size.width / 2) > l[0]){
                horizontal = abs((image_size.width / 2) - l[0]);
                //cout << "Test1:" << horizontal << endl;
            }
            else{
                horizontal = abs(l[0] - (image_size.width / 2));
                //cout << "Test2:" << horizontal << endl;
            }

            if(l[1] > l[3]){
                length = l[1]-l[3];
                vertical = l[3] + (length / 2);
                //cout << "Test3:" << horizontal << endl;
            }
            else if(l[3] > l[1]){
                length = l[3]-l[1];
                vertical = l[1] + (length / 2);
                //cout << "Test4:" << horizontal << endl;

            }
            metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
        }
        else{
            
            if(l[2] > l[0]){
                ankath = l[2]-l[0];
                if(l[1]==l[3]){
                    //wagerechte Linie
                    length = ankath;
                    vertical = l[1];
                    if(l[0] > (image_size.width / 2)){
                        horizontal = abs((l[0] + (ankath / 2)) - (image_size.width / 2));
                        //cout << "Test5:" << horizontal << endl;
                    }
                    else{
                        horizontal = abs((image_size.width / 2) - (l[0] + (ankath / 2)));
                        //cout << "Test6:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                }
                else if(l[1] < l[3]){
                    // Linie von Links unten nach Rechts oben
                    gegekath = l[3]-l[1];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        winkel = atan(gegekath/(double)ankath)*180/PI;
                        length = sqrt(ankath * ankath + gegekath * gegekath);
                        vertical = l[0] + (gegekath / 2);
                        if(l[0] > (image_size.width / 2)){
                            horizontal = abs(((l[0] + (ankath / 2)) - (image_size.width / 2)));
                            //cout << "Test7:" << horizontal << endl;
                        }
                        else{
                            horizontal = abs((image_size.width / 2) - (l[0] + (ankath / 2)));
                            //cout << "Test8:" << horizontal << endl;
                        }
                        metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                        //double verticalspas = (vertical/(image_size.height * (2 / 3)));
                        //metrikcounter = ((1-abs(1-(winkel/75.0)))*(length)+(1-abs(1-(horizontal/(image_size.width / 2)))*(horizontal)+(1-abs(1-verticalspas))*(vertical)));
                        //hier Testen
                    }
                }
                else if(l[1] > l[3]){
                    // Linie von links Oben nach rechts unten
                    gegekath = l[1]-l[3];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        winkel = atan(gegekath/(double)ankath)*180/PI;
                        length = sqrt(ankath * ankath + gegekath * gegekath);
                        vertical = l[3] + (gegekath / 2);
                        if(l[0] > (image_size.width / 2)){
                            horizontal = abs((l[0] + (ankath / 2)) - (image_size.width / 2));
                            //cout << "Test9:" << horizontal << endl;
                        }
                        else{
                            horizontal = abs((image_size.width / 2) - (l[0] + (ankath / 2)));
                            //cout << "Test10:" << horizontal << endl;
                        }
                         metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                    }
                }
            }
            else if(l[2] < l[0]){
                ankath = l[0]-l[2];
                if(l[1]==l[3]){
                    // wagerechte Linie
                    length = ankath;
                    vertical = l[1];
                    if(l[2] > (image_size.width / 2)){
                        horizontal = abs((l[2] + (ankath / 2)) - (image_size.width / 2));
                        //cout << "Test11:" << horizontal << endl;
                    }
                    else{
                        horizontal = abs((image_size.width / 2) - (l[2] + (ankath / 2)));
                        //cout << "Test12:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                }
                else if(l[1] < l[3]){
                    // Linie von links Unten nach rechts Oben
                    gegekath = l[3]-l[1];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        winkel = atan(gegekath/(double)ankath)*180/PI;
                        length = sqrt(ankath * ankath + gegekath * gegekath);
                        vertical = l[1] + (gegekath / 2);
                        if(l[2] > (image_size.width / 2)){
                         horizontal = abs((l[2] + (ankath / 2)) - (image_size.width / 2));
                         //cout << "Test13:" << horizontal << endl;
                        }
                        else{
                          horizontal = abs((image_size.width / 2) - (l[2] + (ankath / 2)));
                          //cout << "Test14:" << horizontal << endl;
                        }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                    }
                }
                else if(l[1] > l[3]){
                    // Linie von links Oben nach rechts Unten
                    gegekath = l[1]-l[3];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        winkel = atan(gegekath/(double)ankath)*180/PI;
                        length = sqrt(ankath * ankath + gegekath * gegekath);
                        vertical = l[3] + ( gegekath / 2);
                        if(l[2] > (image_size.width / 2)){
                            horizontal = abs((l[2] + (ankath / 2)) - (image_size.width / 2));
                            //cout << "Test15:" << horizontal << endl;
                        }
                        else{
                            horizontal = abs((image_size.width / 2) - (l[2] + (ankath / 2)));
                            //cout << "Test16:" << horizontal << endl;
                        }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                    }
                }
            }
        }
    }
    cout << "Metrik:" << metrikcounter << endl;
    return metrikcounter;
}

int main(int argc, char** argv)
{
    // Deklarieren der Ausgabevariablen:
    Mat fdst, dst, cdst, cdstP;
    const char* default_file = argv[1];
    const char* filename = argc >=2 ? argv[1] : default_file;
    // Laden des Bilds:
    Mat src = imread( samples::findFile( filename ), IMREAD_GRAYSCALE );
    // Überprüfung des Ladevorgangs:
    if(src.empty()){
        printf(" Error opening image\n");
        printf(" Program Arguments: [image_name -- default %s] \n", default_file);
        return -1;
    }
    // Bildskalierung:
    cv::resize(src,src,cv::Size(), 0.25 , 0.25);
    // Filtering
    fdst = filter(src, 1);
    // Edge detection
    Canny(fdst, dst, 50, 200, 3);
    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 120, 100, 20 ); // runs the actual detection

    // Zeichnen der Linien
    // ankath = x gerade | gegekath = y gerade
    int ankath,gegekath;
    double winkelfilter = 22.5;
    int mid = (cdstP.size().width)/2;
    double winkel;
    double metrik = line_evaluation(linesP, cdstP.size());

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];

        // line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        // Liniensortierung: (Horizontal wird aussortiert)
        if(l[2]==l[0]){
        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);    
        }
        else{
            
            if(l[2] > l[0]){
            ankath = l[2]-l[0];
                if(l[1]==l[3]){
            
                    }
                else if(l[1] < l[3]){
                gegekath = l[3]-l[1];
                if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
                }
                }
                else if(l[1] > l[3]){
                gegekath = l[1]-l[3];
                if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA); 
                }
                }
            }
            else if(l[2] < l[0]){
            ankath = l[0]-l[2];
                if(l[1]==l[3]){
            
                    }
                else if(l[1] < l[3]){
                gegekath = l[3]-l[1];
                if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA);
                }
                }
                else if(l[1] > l[3]){
                gegekath = l[1]-l[3];
                if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                    line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA); 
                }
                }
            }
        }
        // Mittellinie (Robotermitte)
        line( cdstP, Point( mid, 0), Point( mid, cdstP.size().height), Scalar(0,255,0), 1, LINE_AA);

        // Debugging Area:
        // cout << "Ankath:" << ankath << "  ";
        // cout << "Gegenkath:" << gegekath << " ";
        // cout << "Winkel:" << atan(gegekath/(double)ankath)*180/PI << "\n";
        // cout << "Punkt1:" << Point(l[0],l[1]);
        // cout << " Punkt2:" << Point(l[2],l[3])<< "\n";

        
    }
    cout << "Höhe: " << cdstP.size().height << endl;
    cout << "Breite: " << cdstP.size().width << endl;
    cout << "Winkelfilter: " << winkelfilter << endl;
    // cout << "Metrik:" << metrik << endl;

    // Auswertung:
    // imshow("Orginal", src);
    // Canny:
    // imshow("Canny", dst);
    // Ergebis:
    imshow("Linienerkennung(rot)- Roboterfahrtrichtung(grün)", cdstP);
    // Wait and Exit
    waitKey();
}