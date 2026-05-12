LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY PE IS
    PORT (
        clock                  : IN  std_logic;
		  reset                  : IN  std_logic;
		  ------------------------------------------------
		  en_in_mult             : IN  std_logic;
		  en_in_sum              : IN  std_logic;
		  en_in_sign             : IN  std_logic;
		  en_out_mult            : IN  std_logic;
		  en_out_sum             : IN  std_logic;
		  en_out_sign            : IN  std_logic;
		  flush                  : IN  std_logic;
		  ------------------------------------------------
        a                      : IN  std_logic_vector(7 downto 0);
        b                      : IN  std_logic_vector(7 downto 0);
		  acc                    : OUT std_logic_vector(31 downto 0);
        output_a               : OUT std_logic_vector(7 downto 0);
		  output_b               : OUT std_logic_vector(7 downto 0)
		  --Puntas de Prueba--
		--  result_sum_out         : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
		--  out_signed_16_to_32    : OUT STD_LOGIC_VECTOR (31 DOWNTO 0);
		--  result_mult_out        : OUT STD_LOGIC_VECTOR (15 downto 0)
		  
    );
END ENTITY;

ARCHITECTURE rtl OF PE IS

-------------------------------------------
			 -- Component declarations--+
-------------------------------------------
			 
		  COMPONENT sumador_ip_wrapped
				  PORT (
				  
						  clock    : in  STD_LOGIC;
						  reset  : in  STD_LOGIC;
						  en_in  : in  STD_LOGIC;  -- Enable para entrada
                    en_out : in  STD_LOGIC;  -- Enable para salida
						  flush  : in  STD_LOGIC;
						  a      : in  STD_LOGIC_VECTOR(31 downto 0);
						  b      : in  STD_LOGIC_VECTOR(31 downto 0);
						  s      : out STD_LOGIC_VECTOR(31 downto 0)
						  
				  );
			END COMPONENT;
			
			
		  COMPONENT signed_16_to_32_ip_wrapped
				  PORT (
						  
					  clock         : in  STD_LOGIC;
					  reset         : in  STD_LOGIC;
					  flush         : in  STD_LOGIC;
					  en_in         : in  STD_LOGIC;  -- Enable para entrada
					  en_out        : in  STD_LOGIC;  -- Enable para salida
					  data_16bit    : in  STD_LOGIC_VECTOR(15 downto 0);
					  data_32bit    : out STD_LOGIC_VECTOR(31 downto 0)
						  
				  );
			END COMPONENT;
			
	
			 
			COMPONENT int8_mult_wrapped
				  PORT (
				  
				        clock   : in  STD_LOGIC;
						  reset : in  STD_LOGIC;  -- Activo alto
						  flush : in  STD_LOGIC;
						  en_in     : in  STD_LOGIC;  -- Enable para entrada
                    en_out    : in  STD_LOGIC;  -- Enable para salida
						  a     : in  STD_LOGIC_VECTOR(7 downto 0);
						  b     : in  STD_LOGIC_VECTOR(7 downto 0);
						  p     : out STD_LOGIC_VECTOR(15 downto 0)
				  );
			 END COMPONENT;

    -- Signals to connect components
    SIGNAL sig_data1 : std_logic_vector(31 downto 0);--Salida Multiplicador
	 SIGNAL sig_data1_16b : std_logic_vector(15 downto 0);--Salida Multiplicador
    SIGNAL sig_data2 : std_logic_vector(31 downto 0);--Salida Sumador

BEGIN

    -- Instantiate Component1
   U1_int8_mult: int8_mult_wrapped
        PORT MAP (
		  
		        clock    => clock,
				  reset  => reset,
				  en_in  => en_in_mult,
              en_out => en_out_mult,
				  flush  => flush,
				  a      => a,
				  b      => b,
				  p      => sig_data1_16b
		 
        );
		  
		   -- Instantiate Component1
    U2_int32_add : sumador_ip_wrapped
	 
        PORT MAP (
		  
		     clock   => clock,
			  reset => reset,
			  en_in  => en_in_sum,
           en_out => en_out_sum,
			  flush  => flush,
			  a     => sig_data1,
			  b     => sig_data2,
			  s     => sig_data2

        ); 
		  
	 U4_signed_16_to_32 : signed_16_to_32_ip_wrapped
	 
        PORT MAP (
		  
        clock             => clock,
        reset             => reset,
		  flush             => flush,
		  en_in             => en_in_sign,
        en_out            => en_out_sign,
        data_16bit        => sig_data1_16b,
        data_32bit        => sig_data1
		  
        );

		----------- 
		--OUTPUTS--
		-----------
	acc       <= sig_data2;
	output_a	 <= a; 
	output_b  <= b;
--	result_sum_out <=sig_data2;
--	result_mult_out <=sig_data1_16b;
--	out_signed_16_to_32 <= sig_data1;
		  
		  
END ARCHITECTURE;
