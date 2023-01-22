# Command Based robot framework

* WPILib documentation: 
  * [Command based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
  * [Scheduler](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html)

## Commands / Scheduler state flow

```mermaid
graph TD
    subgraph command
        A[initialize]
        B[execute]
        C{is_finished}
        D[end]

        B --> C
        C -->|True| D
    end

    subgraph scheduler
        Z[schedule]
        Y[run]
        X[interrupt other commands]
        W[initialize]
        

        Z --> A
        Y --> X
        X --> W
        W --> B
        
    end
```