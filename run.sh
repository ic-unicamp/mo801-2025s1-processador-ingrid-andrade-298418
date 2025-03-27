if [ -z "$1" ]; then
    echo "Erro: É preciso especificar o número do teste"
    exit 1
fi

if ! iverilog -o tb *.v; then
    echo "Erro ao compilar com iverilog"
    exit 1
fi

cp test/teste$1.mem memory.mem
./tb > saida.out
sed -i '1,/^\*\*\*.*$/d' saida.out
cp saida.out test/saida$1.out
cp saida.vcd test/saida$1.vcd
rm saida.out saida.vcd memory.mem

if diff test/saida$1.out test/saida$1.ok >/dev/null; then
    echo "OK"
    exit 0
else
    echo "ERRO"
    exit 1
fi
