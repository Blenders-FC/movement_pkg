#ifndef REPEAT_N_TIMES_H
#define REPEAT_N_TIMES_H

#include <control_node.h>
#include <string>

namespace BT
{
class Repeat_n_times : public ControlNode
{
private:
    unsigned int num_repeats_;  // Número de repeticiones (0 = infinito)
    unsigned int current_count_; // Contador actual
    
public:
    // Constructor modificado para aceptar parámetros
    Repeat_n_times(std::string name, unsigned int num_repeats = 0);
    ~Repeat_n_times();

    BT::ReturnStatus Tick() override;
    void Halt() override;
    int DrawType() override;
    
    // Métodos para configurar las repeticiones
    void setNumRepeats(unsigned int num_repeats);
    unsigned int getNumRepeats() const;
};
} // namespace BT

#endif // REPEAT_N_TIMES_H