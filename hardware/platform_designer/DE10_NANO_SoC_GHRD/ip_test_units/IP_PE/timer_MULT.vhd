library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity timer_MULT is
    port(
        clock     : in  std_logic;          
        reset     : in  std_logic;  -- Reset asíncrono
        clear     : in  std_logic;  -- Clear síncrono
        start     : in  std_logic;  -- Pulso de inicio
        done      : out std_logic
    );
end entity;

architecture rtl of timer_MULT is
    type state_type is (IDLE, COUNTING, DONE_ST);
    signal state, next_state : state_type;
    signal count, next_count : unsigned(2 downto 0); -- 2 bits para contar hasta 2
begin
    -- Proceso 1: Memoria de estado y contador (secuencial)
    process(clock, reset)
    begin
        if reset = '1' then
            state <= IDLE;
            count <= (others => '0');
        elsif rising_edge(clock) then
            state <= next_state;
            count <= next_count;
        end if;
    end process;

    -- Proceso 2: Lógica de próximo estado y próximo contador (combinacional)
    process(state, count, start, clear)
    begin
        -- Valores por defecto para evitar latches
        next_state <= state;
        next_count <= count;

        -- Clear síncrono tiene prioridad sobre todo
        if clear = '1' then
            next_state <= IDLE;
            next_count <= (others => '0');
        else
            case state is
                when IDLE =>
                    if start = '1' then
                        next_state <= COUNTING;
                        next_count <= (others => '0');  -- arranca contador en 0
                    else
                        next_state <= IDLE;
                        next_count <= (others => '0');  -- count se mantiene en 0
                    end if;

                when COUNTING =>
                    if count = 4 then          -- 5 ciclo (0,1,2,3,4)
                        next_state <= DONE_ST;
                        next_count <= count;    -- opcional: mantener el valor
                    else
                        next_state <= COUNTING;
                        next_count <= count + 1;
                    end if;

                when DONE_ST =>
                    -- Aquí nos quedamos hasta que llegue clear (ya cubierto arriba)
                    next_state <= DONE_ST;
                    next_count <= count;        -- no importa, pero evitamos latch
            end case;
        end if;
    end process;

    -- Proceso 3: Salida (combinacional, estilo Moore)
    process(state)
    begin
        if state = DONE_ST then
            done <= '1';
        else
            done <= '0';
        end if;
    end process;

end architecture;