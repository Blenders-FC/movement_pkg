#include "movement_pkg/nodes/repeat_n_times.h"
#include "movement_pkg/utils.h"


BT::Repeat_n_times::Repeat_n_times(std::string name, unsigned int num_repeats)
    : ControlNode(name), num_repeats_(num_repeats), current_count_(0) {}

BT::ReturnStatus BT::Repeat_n_times::Tick()
{
    // Reiniciar contador si no está en ejecución
    if (get_status() != BT::RUNNING) {
        current_count_ = 0;
    }
    
    set_status(BT::RUNNING);

    // Verificar si hemos alcanzado el número de repeticiones
    if (num_repeats_ > 0 && current_count_ >= num_repeats_) {
        return BT::SUCCESS;
    }

    // Ejecutar el hijo (asumiendo solo 1 hijo como en tu versión original)
    if (children_nodes_.size() != 1) {
        throw std::runtime_error("RepeatNode must have exactly one child.");
    }

    BT::ReturnStatus child_status = children_nodes_[0]->Tick();

    switch (child_status) {
        case BT::SUCCESS:
            current_count_++;
            // Si aún no alcanzamos el límite o es infinito, seguir repitiendo
            return (num_repeats_ == 0 || current_count_ < num_repeats_) ? 
                   BT::RUNNING : BT::SUCCESS;
        
        case BT::FAILURE:
            return BT::FAILURE;
        
        case BT::RUNNING:
            return BT::RUNNING;
        
        default:
            return BT::IDLE;
    }
}

void BT::Repeat_n_times::Halt()
{
    current_count_ = 0;
    ControlNode::Halt();
}

int BT::Repeat_n_times::DrawType()
{
    return BT::REPEAT;
}

void BT::Repeat_n_times::setNumRepeats(unsigned int num_repeats)
{
    num_repeats_ = num_repeats;
}

unsigned int BT::Repeat_n_times::getNumRepeats() const
{
    return num_repeats_;
}
