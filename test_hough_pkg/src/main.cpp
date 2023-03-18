

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
     cv::boxFilter(src, zdst, -1, cv::Size(4.0,4.0));
        break;
    case 5:
    // Gaussian Filter:
     cv::GaussianBlur(src, zdst, cv::Size(5,5),4.0,2);
        break;
    }
    return zdst;
}

double Metrik(double winkel, int length, int vertical, int horizontal, Size image_size){
    double verticalfactor = (1-abs(1-(vertical/((image_size.height * 2) / 3))));
    double horizontalfactor = (1-abs(1-(horizontal/(image_size.width / 2))));
    double winkelfactor = (abs(1-(winkel/75.0)));
    double metrikcounter = (winkelfactor * length) + (horizontalfactor * horizontal) + (verticalfactor * vertical);
return metrikcounter;
}

double line_evaluation(const vector<Vec4i> linesP, Size image_size){

    int ankath,gegekath;
    double winkel;
    int length;
    int vertical;
    int horizontal;
    int schwerpunktx;
    double metrikcounter = 0.0;
    //cout << "Test:" << endl;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        //cout << "i:" << i << endl;
        // line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        // Liniensortierung: (Horizontal wird aussortiert)
        if(l[2]==l[0]){   
            // Senkrechte Linie
            if((image_size.width / 2) > l[0]){
                horizontal = l[0];
                //cout << "Test1:" << horizontal << endl;
            }
            else{
                horizontal = (image_size.width - l[0]);
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
            winkel = 0.0;
            metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
        }
        else{
            
            if(l[2] > l[0]){
                ankath = l[2]-l[0];
                schwerpunktx = (l[0] + (ankath / 2));
                if(l[1]==l[3]){
                    //wagerechte Linie
                    length = ankath;
                    vertical = l[1];
                    winkel = 0.0;
                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal =  (image_size.width - schwerpunktx);
                        //cout << "Test5:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test6:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                }
                else if(l[1] < l[3]){
                    // Linie von Links unten nach Rechts oben
                    gegekath = l[3]-l[1];
                    winkel = atan(gegekath/(double)ankath)*180/PI;
                    length = sqrt(ankath * ankath + gegekath * gegekath);
                    vertical = l[1] + (gegekath / 2);

                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal = (image_size.width - schwerpunktx);
                        //cout << "Test7:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test8:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);

                    
                }
                else if(l[1] > l[3]){
                    // Linie von links Oben nach rechts unten
                    gegekath = l[1]-l[3];
                    winkel = atan(gegekath/(double)ankath)*180/PI;
                    length = sqrt(ankath * ankath + gegekath * gegekath);
                    vertical = l[3] + (gegekath / 2);

                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal = (image_size.width - schwerpunktx);
                        //cout << "Test9:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test10:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                    
                }
            }
            else if(l[2] < l[0]){
                ankath = l[0]-l[2];
                schwerpunktx = (l[2] + (ankath / 2));
                if(l[1]==l[3]){
                    // wagerechte Linie
                    length = ankath;
                    vertical = l[1];
                    winkel = 0.0;
                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal = (image_size.width - schwerpunktx);
                        //cout << "Test11:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test12:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                }
                else if(l[1] < l[3]){
                    // Linie von links Unten nach rechts Oben
                    gegekath = l[3]-l[1];
                    winkel = atan(gegekath/(double)ankath)*180/PI;
                    length = sqrt(ankath * ankath + gegekath * gegekath);
                    vertical = l[1] + (gegekath / 2);

                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal = (image_size.width - schwerpunktx);
                        //cout << "Test13:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test14:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                    
                }
                else if(l[1] > l[3]){
                    // Linie von links Oben nach rechts Unten
                    gegekath = l[1]-l[3];
                    winkel = atan(gegekath/(double)ankath)*180/PI;
                    length = sqrt(ankath * ankath + gegekath * gegekath);
                    vertical = l[3] + (gegekath / 2);

                    if(schwerpunktx > (image_size.width / 2)){
                        horizontal = (image_size.width - schwerpunktx);
                        //cout << "Test15:" << horizontal << endl;
                    }
                    else{
                        horizontal = schwerpunktx;
                        //cout << "Test16:" << horizontal << endl;
                    }
                    metrikcounter += Metrik(winkel,length,vertical,horizontal,image_size);
                
                }
            }
        }
    }
    // cout << "Metrik: " << metrikcounter << endl;
    return metrikcounter;
}

void line_filter(const vector<Vec4i> linesP, vector<Vec4i>& linesF_, Size image_size){


    int leftthreshhold = ((image_size.width * 20) / 100);
    int rightthreshhold = ((image_size.width * 80) / 100);

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        int x_value;
        // l[0] l[2]
        if(l[0] > l[2]){
            x_value = (l[0] + l[2]) / 2;
        }
        else if(l[0] < l[2]){
            x_value = (l[2] + l[0]) / 2;
        }
        
        if(x_value < leftthreshhold){

            linesF_.push_back(linesP[i]);

        }
        else if(x_value > rightthreshhold){

            linesF_.push_back(linesP[i]);

        }
        
    }


}

//Funktion für Linienlöschen (in gewissen Toleranzbereich)

//Funktion für Linienzeichnen ()

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
    vector<Vec4i> linesF;
    line_filter(linesP,linesF, cdstP.size());
    int printedlines = 0;

    // Mittellinie (Robotermitte)
    line( cdstP, Point( mid, 0), Point( mid, cdstP.size().height), Scalar(0,255,0), 1, LINE_AA);
    // Schwellwert y-Richtung
    line( cdstP, Point( 0, ((cdstP.size().height * 2) / 3)), Point( cdstP.size().width, ((cdstP.size().height * 2) / 3)), Scalar(0,255,0), 1, LINE_AA);
    // Schwellwert Linienfilter Links
    line( cdstP, Point( ((cdstP.size().width * 20) / 100), 0), Point( ((cdstP.size().width * 20) / 100), cdstP.size().height), Scalar(255,0,0), 1, LINE_AA);
    // Schwellwert Linienfilter Rechts
    line( cdstP, Point( ((cdstP.size().width * 80) / 100), 0), Point( ((cdstP.size().width * 80) / 100), cdstP.size().height), Scalar(255,0,0), 1, LINE_AA);


    for( size_t i = 0; i < linesF.size(); i++ )
    {
        Vec4i l = linesF[i];

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
                        printedlines = printedlines + 1;
                    }
                }
                else if(l[1] > l[3]){
                    gegekath = l[1]-l[3];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA); 
                        printedlines = printedlines + 1;
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
                        printedlines = printedlines + 1;
                    }
                }
                else if(l[1] > l[3]){
                    gegekath = l[1]-l[3];
                    if( atan(gegekath/(double)ankath)*180/PI >= winkelfilter){
                        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, LINE_AA); 
                        printedlines = printedlines + 1;
                    }
                }
            }
        }
        
    }
    
    cout << "Metrik: " << metrik << endl;
    cout << "ges. Linienanzahl: " << linesP.size() << endl;
    cout << "gef. Linien: " << linesF.size() << endl;
    cout << "gez. Linien: " << printedlines << endl;
    cout << "Höhe: " << cdstP.size().height << endl;
    cout << "Breite: " << cdstP.size().width << endl;
    cout << "Winkelfilter: " << winkelfilter << endl;


    // Auswertung:
    // imshow("Orginal", src);
    // Canny:
    // imshow("Canny", dst);
    // Ergebis:
    imshow("Linienerkennung(rot)- Roboterfahrtrichtung(grün)", cdstP);
    // Wait and Exit
    waitKey();
}