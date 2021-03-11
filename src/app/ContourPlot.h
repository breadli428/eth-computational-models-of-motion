#include <ObjectiveFunction.h>
#include <colormap.h>

using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Vector2i;

struct ContourPlot {
    double min, max;    // min/max values for plot
    Vector2d from, to;  // plot space dimensions

    Vector2i resOld = {0,0};

    int nContours = 20;
    double * data = nullptr;
    unsigned char * imgData = nullptr;
    bool showContourEdges = false;

    void generate(const ObjectiveFunction *obj, Vector2i res, bool recompute = false) {

        if(res != resOld){
            if(imgData != nullptr)
                free(imgData);
            imgData = new unsigned char[res.x()*res.y()*4];

            if(data != nullptr)
                free(data);
            data = new double[res.x()*res.y()];
            resOld = res;
        }

        int w = res.x(), h = res.y();
//        size = w;

        // find min/max
        if(recompute){
            min = HUGE_VAL; max = -HUGE_VAL;
            for (int j = 0; j < h; ++j) {
                for (int i = 0; i < w; ++i) {
                    double f = obj->evaluate(fromImgtoPlot({i, j}));
                    min = std::min(min, f);
                    max = std::max(max, f);
                }
            }
        }

        double * dataPtr = data;
        unsigned char* px = imgData;
        for (int j = 0; j < h; ++j) {
            for (int i = 0; i < w; ++i) {
                // sample function at i,j
                double f = obj->evaluate(fromImgtoPlot({i, j}));
                *(dataPtr++) = f;

                // use log-scale, +1 to prevent log(0)
                f = log10(f - min + 1) / log10(max - min + 1);

                // put into 'buckets'
                f = floor(f*(double)nContours) / (double)nContours;

                // map to a color
                float r, g, b;
                colorMapColor(f, r, g, b);
                px[0] = (unsigned char)(255.f*r);
                px[1] = (unsigned char)(255.f*g);
                px[2] = (unsigned char)(255.f*b);
                px[3] = 150;
                px += 4; // advance to next pixel
            }
        }

        if(showContourEdges){
            px = imgData;
            for (int j = 0; j < h; ++j) {
                for (int i = 0; i < w; ++i) {
                    for (int k = 0; k < 3; ++k) {
                        if((i<w-1 && px[k] != px[4+k]) || (j<h-1 && px[k] != px[4*w+k])){
                            px[0] = 0;
                            px[1] = 0;
                            px[2] = 0;
                            px[3] = 150;
                            break;
                        }
                    }
                    px += 4;
                }
            }
        }
    }

    void translate(const ObjectiveFunction *obj, Vector2i res, int dw, int dh, bool recompute = false) {

//        if(resolution != res)
        {
            generate(obj, res, recompute);
        }

        int w = res.x(), h = res.y();
//        size = w;

        Vector2d dx = Vector2d(dw, -dh)/(double)w;
        from -= dx.cwiseProduct(to-from);
        to -= dx.cwiseProduct(to-from);

        // find min/max
        if(recompute){
            min = HUGE_VAL; max = -HUGE_VAL;
            for (int j = 0; j < res.y(); ++j) {
                for (int i = 0; i < res.x(); ++i) {
                    double f = obj->evaluate(fromImgtoPlot({i, j}));
                    min = std::min(min, f);
                    max = std::max(max, f);
                }
            }
        }

        int wm = std::max(0, dw);
        int wp = std::min(w, w+dw);
        int hm = std::max(0, dh);
        int hp = std::min(h, h+dh);

        double * dataNew = new double[w*h];
        double * dataNewPtr = dataNew;
        for (int j = 0; j < h; ++j) {
            for (int i = 0; i < w; ++i) {

                if(i >= wm && i < wp && j >= hm && j < hp){
                    int idx = (w*(j-dh) + i-dw);
                    *(dataNewPtr++) = data[idx];
                }
                else{
                    *(dataNewPtr++) = obj->evaluate(fromImgtoPlot({i, j}));
                }
            }
        }
        delete data;
        data = dataNew;

        dataNewPtr = dataNew;
        unsigned char* px = imgData;
        for (int j = 0; j < h; ++j) {
            for (int i = 0; i < w; ++i) {
                // sample function at i,j
                double f = *(dataNewPtr++);

                // use log-scale, +1 to prevent log(0)
                f = log10(f - min + 1) / log10(max - min + 1);

                // put into 'buckets'
                f = floor(f*(double)nContours) / (double)nContours;

                // map to a color
                float r, g, b;
                colorMapColor(f, r, g, b);
                px[0] = (unsigned char)(255.f*r);
                px[1] = (unsigned char)(255.f*g);
                px[2] = (unsigned char)(255.f*b);
                px[3] = 150;
                px += 4; // advance to next pixel
            }
        }

        if(showContourEdges){
            px = imgData;
            for (int j = 0; j < h; ++j) {
                for (int i = 0; i < w; ++i) {
                    for (int k = 0; k < 3; ++k) {
                        if((i < w-1 && px[k] != px[4+k]) || (j < h-1 && px[k] != px[4*w+k])){
                            px[0] = 0;
                            px[1] = 0;
                            px[2] = 0;
                            px[3] = 150;
                            break;
                        }
                    }
                    px += 4;
                }
            }
        }
    }

    // i,j: resolution space
    Vector2d fromImgtoPlot(const Vector2i &ijImg) const {
        Vector2d x = {(double)ijImg.x()/(double)resOld.x(), -(double)ijImg.y()/(double)resOld.y()};
//        x[0] = ((double)i/(double)w - translation[0])*zoom;
//        x[1] = (-(double)j/(double)h - translation[1])*zoom;

        return from + x.cwiseProduct(to-from);
    }

    // s: plot space
    double fromPlotToImg(double s, int dim) const {
        return (s - from[dim])/(to[dim]-from[dim])  * (double)((dim == 0) ? resOld.x() : -resOld.y());
//        return (s/zoom + translation[dim]) * (double)((dim == 0) ? size : -size);
    }

    Vector2f fromPlotToImg(const Vector2d &s) const {
        return {fromPlotToImg(s[0], 0), fromPlotToImg(s[1], 1)};
//        return (s/zoom + translation[dim]) * (double)((dim == 0) ? size : -size);
    }
};
