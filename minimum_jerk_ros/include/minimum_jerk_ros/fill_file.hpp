#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

namespace minimum_jerk
{
    class FilFile
    {
    public:
        template <typename T>
        static void fil_file_rotation(std::vector<double> values1, std::vector<T> values2, std::string file_name)
        {
            int dup_stdout = dup(STDOUT_FILENO);
            int fd = creat(file_name.c_str(), O_CREAT | O_WRONLY | O_TRUNC);

            dup2(fd, 1);
            int i = 0;
            for (T val : values2)
            {
                printf("%f %f\n", values1.at(i), val.get_theta());
                i++;
            }
            dup2(dup_stdout, 1);
            close(fd);
        }
        template <typename T>
        static void fil_file_translation(std::vector<double> values1, std::vector<T> values2, std::string file_name)
        {
            int dup_stdout = dup(STDOUT_FILENO);
            int fd = creat(file_name.c_str(), O_CREAT | O_WRONLY | O_TRUNC);

            dup2(fd, 1);
            int i = 0;
            for (T val : values2)
            {
                printf("%f %f\n", values1.at(i), val.get_x());
                i++;
            }
            dup2(dup_stdout, 1);
            close(fd);
        }
        static int open_file(std::string name)
        {
            int fd = creat(name.c_str(), O_CREAT | O_WRONLY | O_TRUNC);
            return fd;
        }
        static void fil_one_ligne(int fd, int i, double value)
        {
            int dup_stdout = dup(STDOUT_FILENO);
            dup2(fd, 1);
            printf("%i %f\n", i, value);
            dup2(dup_stdout, 1);
        }
        static void fil_one_ligne(int fd, int i, double value1, double value2)
        {
            int dup_stdout = dup(STDOUT_FILENO);
            dup2(fd, 1);
            printf("%i %f %f\n", i, value1, value2);
            dup2(dup_stdout, 1);
        }
        static void close_file(int fd)
        {
            close(fd);
        }
    };
}